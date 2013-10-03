#include <ros/ros.h>

#include <stdio.h>
#include <malloc.h>
#include <GL/osmesa.h>
#include <GL/glu.h>
#include <GL/glut.h>


#include <arm_navigation_msgs/GetPlanningScene.h>
#include <planning_environment/models/model_utils.h>
#include <planning_environment/models/collision_models.h>
#include <geometric_shapes/shapes.h>

#include <sensor_msgs/CameraInfo.h>

#include <tf/transform_listener.h>
#include <image_geometry/pinhole_camera_model.h>

#include <cv.h>
#include <highgui.h>


#include <set>

#include <camera_helpers/OffscreenRenderService.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

using namespace std ;

class CollisionModelRenderer {
public:
    CollisionModelRenderer(): padding(0.001) {}
    ~CollisionModelRenderer() ;

    bool init(const image_geometry::PinholeCameraModel &cm_ ) ;
    cv::Mat render(const tf::Transform &tf) ;

    void ignoreLink(const string &link) {
        ignored.insert(link) ;
    }

private:

    bool initMesa() ;
    void deinitMesa() ;
    void initGL(const tf::Transform &tf) ;

    void renderObjects() ;
    cv::Mat readBuffer() ;

    GLfloat *buffer;
    OSMesaContext ctx ;

    double padding ;

    set<string> ignored ;


    boost::shared_ptr<planning_environment::CollisionModels> cm_ ;
    planning_models::KinematicState *state_ ;

    int width, height ;
    float fy ;

};

cv::Mat CollisionModelRenderer::readBuffer()
{
    const GLfloat *ptr = buffer;

    cv::Mat_<uchar> res(height, width) ;

    for (int y=height-1, i=0; y>=0; y--, i++)
    {
        for (int x=0, j=0; x<width; x++, j++)
        {
            int idx = (y*width + x) * 4;
            float a = ptr[idx+3] ;

            if ( a > 0.0 )
                res[i][j] = 255 ;
            else
                res[i][j] = 0 ;
        }

    }

    return res ;
}

bool CollisionModelRenderer::initMesa()
{
    /* Create an RGBA-mode context */
 #if OSMESA_MAJOR_VERSION * 100 + OSMESA_MINOR_VERSION >= 305
    /* specify Z, stencil, accum sizes */
    ctx = OSMesaCreateContextExt( GL_RGBA,  16, 0, 0, NULL );
 #else
    ctx = OSMesaCreateContext( GL_RGBA, NULL );
 #endif
    if ( !ctx ) return false ;

    /* Allocate the image buffer */
    buffer = (GLfloat *) malloc( width * height * 4 * sizeof(GLfloat));
    if (!buffer) return false ;

    /* Bind the buffer to the context and make it current */
    if  (!OSMesaMakeCurrent( ctx, buffer, GL_FLOAT, width, height )) return false ;
}

void CollisionModelRenderer::deinitMesa()
{
    free( buffer );
    OSMesaDestroyContext( ctx );
}


void CollisionModelRenderer::initGL(const tf::Transform &tf)
{
    glClearColor(0.0, 0.0, 0.0, 0.0) ;

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    glColor4f(1.0, 1.0, 1.0, 1.0) ;

   // projection

    glMatrixMode(GL_PROJECTION);

    const double zNear = 0.001 ;
    const double zFar = 100.0 ;

    double fovy = 2 * atan( height / fy/2.0)  ;

    gluPerspective( fovy * 180/M_PI, (double)width / height, zNear, zFar );
    // viewing transformation

    glMatrixMode(GL_MODELVIEW) ;
    glLoadIdentity();

    GLdouble m[16] = {  1,  0, 0, 0,
                        0, -1, 0, 0,
                        0, 0, -1, 0,
                        0, 0, 0, 1 };

    glLoadMatrixd(m) ;

    tf.getOpenGLMatrix(m);
    glMultMatrixd(m) ;
}

static void renderBox(double sx, double sy, double sz)
{

    glBegin(GL_QUADS);

    glVertex3f( sx, sy, -sz);
    glVertex3f(-sx, sy, -sz);
    glVertex3f(-sx, sy,  sz);
    glVertex3f( sx, sy,  sz);

    glVertex3f( sx, -sy,  sz);
    glVertex3f(-sx, -sy,  sz);
    glVertex3f(-sx, -sy, -sz);
    glVertex3f( sx, -sy, -sz);

    glVertex3f( sx,  sy, sz);
    glVertex3f(-sx,  sy, sz);
    glVertex3f(-sx, -sy, sz);
    glVertex3f( sx, -sy, sz);

    glVertex3f( sx, -sy, -sz);
    glVertex3f(-sx, -sy, -sz);
    glVertex3f(-sx,  sy, -sz);
    glVertex3f( sx,  sy, -sz);

    glVertex3f(-sx,  sy,  sz);
    glVertex3f(-sx,  sy, -sz);
    glVertex3f(-sx, -sy, -sz);
    glVertex3f(-sx, -sy,  sz);

    glVertex3f(sx,  sy, -sz);
    glVertex3f(sx,  sy,  sz);
    glVertex3f(sx, -sy,  sz);
    glVertex3f(sx, -sy, -sz);

    glEnd();
}


static void renderMesh(const shapes::Mesh *mesh)
{
    glBegin(GL_TRIANGLES);

    for(int i=0 ; i<mesh->triangleCount ; i++ )
    {
        int v1 = mesh->triangles[3*i] ;
        int v2 = mesh->triangles[3*i+1] ;
        int v3 = mesh->triangles[3*i+2] ;

        glVertex3d( mesh->vertices[3*v1],  mesh->vertices[3*v1+1], mesh->vertices[3*v1+2]);
        glVertex3d( mesh->vertices[3*v2],  mesh->vertices[3*v2+1], mesh->vertices[3*v2+2]);
        glVertex3d( mesh->vertices[3*v3],  mesh->vertices[3*v3+1], mesh->vertices[3*v3+2]);

    }
    glEnd();
}



static void renderCollisionBody(const shapes::Shape *shape, double scale, double padding)
{

    switch (shape->type)
    {
        case shapes::SPHERE:
        case shapes::CYLINDER:
            break;
        case shapes::BOX:
        {
            const double *size = static_cast<const shapes::Box*>(shape)->size;

            renderBox(size[0] * scale/2 + padding, size[1] * scale/2 + padding, size[2] * scale/2  + padding) ;


        }
        break;

        case shapes::MESH:
        {
            const shapes::Mesh *mesh = static_cast<const shapes::Mesh*>(shape);

            renderMesh(mesh) ;

        }
        break ;

        default:
        break;
    }


}

void CollisionModelRenderer::renderObjects()
{
    const collision_space::EnvironmentModel *cs = cm_->getCollisionSpace() ;

    const planning_models::KinematicModel *robot = cs->getRobotModel() ;

    const std::vector<planning_models::KinematicModel::LinkModel *> links = robot->getLinkModels() ;

    for(int i=0 ; i<links.size() ; i++)
    {
        const planning_models::KinematicModel::LinkModel *link_ = links[i] ;

        string name = link_->getName() ;

        if ( ignored.count(name) > 0 ) continue ;

        const planning_models::KinematicState::LinkState* link_state_ = state_->getLinkState(name);

        const shapes::Shape* shape = link_->getLinkShape();

        tf::Transform gt =   link_state_->getGlobalCollisionBodyTransform();

        if ( shape )
        {
            glPushMatrix();

            tfScalar m[16] ;
            gt.getOpenGLMatrix(m);

            glMultMatrixd(m);

            renderCollisionBody(shape, 1.0, padding) ;

            glPopMatrix() ;
        }
    }

}

bool CollisionModelRenderer::init(const image_geometry::PinholeCameraModel &cmodel)
{

    ros::service::waitForService("/environment_server/get_planning_scene");

    arm_navigation_msgs::GetPlanningScene planning_scene;

    if (!ros::service::call("/environment_server/get_planning_scene", planning_scene)) {
        ROS_ERROR("Can't get planning scene");
        return false ;
    }

    cm_.reset(new planning_environment::CollisionModels("/robot_description")) ;
    state_ = cm_->setPlanningScene(planning_scene.response.planning_scene);

    width = cmodel.width() ;
    height = cmodel.height() ;
    fy = cmodel.fy() ;

    ros::NodeHandle nh("~") ;

    string ignored_ ;
    nh.getParam("ignore_links", ignored_) ;

    boost::split(ignored, ignored_, boost::is_any_of(";/ "));

    return true ;

}


CollisionModelRenderer::~CollisionModelRenderer()
{

    cm_->revertPlanningScene(state_);
}

cv::Mat CollisionModelRenderer::render(const tf::Transform &tf)
{
    initMesa() ;
    initGL(tf) ;
    renderObjects() ;
    cv::Mat im = readBuffer() ;
    deinitMesa() ;

    return im ;
}

CollisionModelRenderer *grdr ;
string link_name ;
ros::ServiceServer server ;
ros::Subscriber camera_sub ;


bool do_render(camera_helpers::OffscreenRenderService::Request &req,  camera_helpers::OffscreenRenderService::Response &res)
{
    tf::TransformListener listener ;

    tf::StampedTransform transform;

    try {
        ros::Time ts(0) ;

        listener.waitForTransform(link_name, "base_link", ts, ros::Duration(1) );
        listener.lookupTransform(link_name, "base_link", ts, transform);

        cv::Mat mask = grdr->render(transform) ;

        cv_bridge::CvImage cvi ;
        cvi.header.stamp = ros::Time::now() ;
        cvi.header.frame_id = link_name;
        cvi.encoding = "mono8";
        cvi.image = mask;

        cvi.toImageMsg(res.mask);

        return true ;

    } catch (tf::TransformException ex) {
        ROS_ERROR("%s",ex.what());
        return false ;
    }
}


void camera_info_callback(const sensor_msgs::CameraInfoConstPtr &cinfo)
{

    image_geometry::PinholeCameraModel camera_model ;
    camera_model.fromCameraInfo(cinfo) ;

    grdr = new CollisionModelRenderer ;
    grdr->init(camera_model) ;

    // Register the service with the master
    ros::NodeHandle nh("~") ;
    server = nh.advertiseService("render", &do_render);

    camera_sub.shutdown() ;
}



int main(int argc, char *argv[])
{
    ros::init(argc, argv, "offscreen_render") ;

    ros::NodeHandle nh("~") ;

    string prefix ;

    nh.getParam("camera_info_topic", prefix) ;
    nh.getParam("camera_link", link_name) ;

    camera_sub = nh.subscribe<sensor_msgs::CameraInfo>(prefix, 1, camera_info_callback ) ;

    ros::spin() ;

}
