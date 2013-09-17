#include <ros/ros.h>

#include <stdio.h>
#include <malloc.h>
#include <GL/osmesa.h>
#include <GL/glu.h>
#include <GL/glut.h>

#include <kinematics_base/kinematics_base.h>
#include <arm_navigation_msgs/GetPlanningScene.h>
#include <planning_environment/models/model_utils.h>
#include <planning_environment/models/collision_models.h>
#include <geometric_shapes/shapes.h>

#include <sensor_msgs/CameraInfo.h>

#include <tf/transform_listener.h>
#include <image_geometry/pinhole_camera_model.h>

#include <cv.h>
#include <highgui.h>

#include <robot_helpers/Utils.h>
#include <set>

using namespace std ;

#define WIDTH 400
#define HEIGHT 400


class CollisionModelRenderer {
public:
    CollisionModelRenderer(): padding(0.001) {}
    ~CollisionModelRenderer() ;

    bool init() ;
    cv::Mat render(const sensor_msgs::CameraInfoConstPtr &cinfo, const tf::Transform &tf) ;

    void ignoreLink(const string &link) {
        ignored.insert(link) ;
    }

private:

    bool initMesa(int width, int height) ;
    void deinitMesa() ;
    void initGL(const sensor_msgs::CameraInfoConstPtr &cinfo, const tf::Transform &tf) ;

    void renderObjects() ;
    cv::Mat readBuffer(int width, int height) ;

    GLfloat *buffer;
    OSMesaContext ctx ;

    double padding ;

    set<string> ignored ;

    boost::shared_ptr<planning_environment::CollisionModels> cm_ ;
    planning_models::KinematicState *state_ ;



};

cv::Mat CollisionModelRenderer::readBuffer(int width, int height)
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


           // r = (int) (ptr[i+0] * 255.0);
          //  g = (int) (ptr[i+1] * 255.0);
          //  b = (int) (ptr[i+2] * 255.0);

        }

    }

    return res ;
}

bool CollisionModelRenderer::initMesa(int width, int height)
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


    //cv::Mat buf = readBuffer(buffer, WIDTH, HEIGHT) ;

   // cv::imwrite("/tmp/oo.png", buf) ;

    /* free the image buffer */
    free( buffer );

    /* destroy the context */
    OSMesaDestroyContext( ctx );

}



void CollisionModelRenderer::initGL(const sensor_msgs::CameraInfoConstPtr &cinfo, const tf::Transform &tf)
{
    glClearColor(0.0, 0.0, 0.0, 0.0) ;

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    glColor4f(1.0, 1.0, 1.0, 1.0) ;

   // projection

    glMatrixMode(GL_PROJECTION);

    const double fieldOfView = M_PI / 4.0;
    const double zNear = 0.001 ;
    const double zFar = 100.0 ;

    float width = cinfo->width ;
    float height = cinfo->height ;

    image_geometry::PinholeCameraModel cm ;
    cm.fromCameraInfo(cinfo) ;

    double fovy = 2 * atan( cinfo->height / cm.fy()/2.0)  ;


    gluPerspective( fovy * 180/M_PI, (double)width / height, zNear, zFar );
    // viewing transformation

    glMatrixMode(GL_MODELVIEW) ;
    glLoadIdentity();
/*
    GLdouble m[16] = {  0, 1, 0, 0,
                        1, 0, 0, 0,
                        0, 0, -1, 0,
                        0, 0, 0, 1 };
  */
    GLdouble m[16] = {  1,  0, 0, 0,
                        0, -1, 0, 0,
                        0, 0, -1, 0,
                        0, 0, 0, 1 };

    gluLookAt(0, 0.5, 1.3, 0, -5, 1.3, 0, 0, 1) ;

return ;
    glGetDoublev(GL_MODELVIEW_MATRIX, m);

    glLoadMatrixd(m) ;
/*
   // GLdouble m[16] = { 1, 0, 0, 0, 0, 0, -1, 0, 0, 1, 0, 0, 0, 0, 0, 1 };
    GLdouble m[16] = { 1, 0, 0, 0,
                       0, 0, 1, 0,
                       0, -1, 0, 0,
                       0, 0, 0, 1 };
    glMultMatrixd(m) ;


    glMultMatrixd(m) ;
*/

    //gluLookAt(0, 0.5, 1.3, 0, -5, 1.3, 0, 0, 1) ;



    //glGetDoublev(GL_MODELVIEW_MATRIX, m);

            tf.getOpenGLMatrix(m);
            glMultMatrixd(m) ;



    tf.getOpenGLMatrix(m);
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
            break;
        case shapes::BOX:
        {
            const double *size = static_cast<const shapes::Box*>(shape)->size;

            renderBox(size[0] * scale/2 + padding, size[1] * scale/2 + padding, size[2] * scale/2  + padding) ;
        }
        break;
        case shapes::CYLINDER:
        {
            double r2 = static_cast<const shapes::Cylinder*>(shape)->radius * scale + padding;
//            btshape = dynamic_cast<btCollisionShape*>(new btCylinderShapeZ(btVector3(r2, r2, static_cast<const shapes::Cylinder*>(shape)->length * scale / 2.0 + padding)));
        }
        break;
        case shapes::MESH:
        {
            const shapes::Mesh *mesh = static_cast<const shapes::Mesh*>(shape);

            renderMesh(mesh) ;
  /*
            for (unsigned int i = 0 ; i < mesh->vertexCount ; ++i)
                btmesh->addPoint(btVector3(mesh->vertices[3*i], mesh->vertices[3*i + 1], mesh->vertices[3*i + 2]));

            btmesh->setLocalScaling(btVector3(scale, scale, scale));
            btmesh->setMargin(padding + 0.0001); // we need this to be positive
            btshape = dynamic_cast<btCollisionShape*>(btmesh);
            */
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

bool CollisionModelRenderer::init()
{
    ros::service::waitForService("/environment_server/set_planning_scene_diff");

    arm_navigation_msgs::GetPlanningScene planning_scene;

    if (!ros::service::call("/environment_server/set_planning_scene_diff", planning_scene)) {
        ROS_ERROR("Can't get planning scene");
        return false ;
    }


    cm_.reset(new planning_environment::CollisionModels("robot_description")) ;
    state_ = cm_->setPlanningScene(planning_scene.response.planning_scene);

    return true ;

}


CollisionModelRenderer::~CollisionModelRenderer()
{

    cm_->revertPlanningScene(state_);
}

cv::Mat CollisionModelRenderer::render(const sensor_msgs::CameraInfoConstPtr &cinfo, const tf::Transform &tf)
{
    int w = cinfo->width, h = cinfo->height ;

    initMesa(w, h) ;
    initGL(cinfo, tf) ;
    renderObjects() ;
    cv::Mat im = readBuffer(w, h) ;
    deinitMesa() ;

    return im ;
}

CollisionModelRenderer *grdr ;
string link_name = "xtion3_rgb_optical_frame";


void camera_info_callback(const sensor_msgs::CameraInfoConstPtr &cinfo)
{

    tf::TransformListener listener ;

    tf::StampedTransform transform;

    try {
        listener.waitForTransform(link_name, "base_link", ros::Time(0), ros::Duration(1) );
        listener.lookupTransform(link_name, "base_link", ros::Time(0), transform);

        cv::Mat res = grdr->render(cinfo, transform) ;
        cv::imwrite("/tmp/im.png", res) ;


    } catch (tf::TransformException ex) {
        ROS_ERROR("%s",ex.what());
    }
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "ofrender") ;

    ros::NodeHandle nh ;



    robot_helpers::MoveRobot mv ;
//    robot_helpers::moveGripperPointingDown(mv, "r1", 0.3, -0.7, 0.4) ;


    grdr = new CollisionModelRenderer() ;

    if ( ! grdr->init() ) return 0 ;

    grdr->ignoreLink("certh_floor");
    grdr->ignoreLink("certh_roof");
    grdr->ignoreLink("certh_wall_1");
    grdr->ignoreLink("certh_wall_2");
    grdr->ignoreLink("certh_wall_3");
    grdr->ignoreLink("certh_wall_4");
    grdr->ignoreLink("camera_stick");


    string prefix = "xtion3" ;

    sensor_msgs::CameraInfoConstPtr tmp_camera ;

    ros::Subscriber  camera_sub = nh.subscribe<sensor_msgs::CameraInfo>("/" + prefix + "/depth_registered/camera_info", 1, camera_info_callback ) ;

    ros::spin() ;


}
