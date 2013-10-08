#include <ros/ros.h>

#include <robot_helpers/Robot.h>
#include <robot_helpers/Utils.h>
#include <robot_helpers/Geometry.h>

#include <camera_helpers/OpenNICapture.h>
#include <pcl/io/pcd_io.h>
#include <highgui.h>

#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>

using namespace std;
using namespace robot_helpers ;
using namespace Eigen ;

class RotateAndGrab {
public:

    RotateAndGrab(const std::string &camera_, const std::string &arm_) ;
    ~RotateAndGrab() ;

    void init(const Vector3d &pos) ;

    void rotate(double theta) ;

protected:

    virtual void process( const pcl::PointCloud<Eigen::MatrixXf> &pc, const cv::Mat &clr, const cv::Mat &depth, const image_geometry::PinholeCameraModel &cm,
                         const ros::Time &ts, Eigen::Affine3d &tip_pose_in_camera_frame) {}


    void startCapture() ;
    void doCapture() ;
    void stopCapture() ;

    camera_helpers::OpenNICaptureRGBD *cap ;

    std::string camera, arm ;
    bool captureStoped ;
    boost::mutex mutex ;
    boost::thread capture_thread ;
    int counter ;
    MoveRobot cmove ;
};


RotateAndGrab::RotateAndGrab(const string &camera_, const string &arm_):  camera(camera_), arm(arm_) {
  cmove.setServoMode(false);
  cap = new camera_helpers::OpenNICaptureRGBD(camera) ;
}

void RotateAndGrab::init(const Vector3d &pos)
{

    moveGripper(cmove, arm, pos, lookAt(Vector3d(0, 0, -1), 0)) ;

cap->connect() ;

    cout << "connected" << endl ;

    cmove.actionStarted.connect(boost::bind(&RotateAndGrab::startCapture, this)) ;
    cmove.actionCompleted.connect(boost::bind(&RotateAndGrab::stopCapture, this)) ;

}

void RotateAndGrab::rotate(double theta)
{
    rotateGripper(cmove, arm, theta) ;

    while (!captureStoped ) ;

    capture_thread.join() ;
}

void RotateAndGrab::startCapture()
{
    captureStoped = false ;
    counter = 0 ;
    capture_thread = boost::thread(boost::bind(&RotateAndGrab::doCapture, this)) ;
}

void RotateAndGrab::doCapture()
{
    bool _stoped = false ;


    do {

        {
            boost::mutex::scoped_lock lock_(mutex) ;
            _stoped = captureStoped ;
        }

        if ( !_stoped )
        {
            cv::Mat clr, depth ;
            ros::Time ts ;
            image_geometry::PinholeCameraModel cm;

            if ( cap->grab(clr, depth, ts, cm) )
            {

                cout << counter++ << endl ;
                ros::Duration(0.5).sleep() ;

            }

        }

    } while ( !_stoped ) ;

}

void RotateAndGrab::stopCapture()
{
    boost::mutex::scoped_lock lock_(mutex) ;
    captureStoped = true ;


}

RotateAndGrab::~RotateAndGrab()
{
    cap->disconnect() ;
    delete cap ;
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "rotate_and_grab_example");
    ros::NodeHandle nh ;

    RotateAndGrab rg("xtion2", "r1") ;
    rg.init(Vector3d(0.0, -1.2, 0.8)) ;
    rg.rotate(M_PI/6) ;
}
