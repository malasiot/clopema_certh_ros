#include <ros/ros.h>

#include <robot_helpers/Robot.h>
#include <robot_helpers/Utils.h>

#include <camera_helpers/OpenNICapture.h>
#include <pcl/io/pcd_io.h>
#include <highgui.h>

#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>

#include <boost/thread/mutex.hpp>

using namespace std;
using namespace robot_helpers ;
using namespace Eigen ;

class RotateAndGrab {
public:

    RotateAndGrab(const string &camera_, const string &arm_): cap(camera_), camera(camera_), arm(arm_) {
        cmove.setServoMode(false);
        counter = 0 ;
    }

    void init(const Vector3d &pos)
    {
        moveGripperPointingDown(cmove, arm, pos.x(), pos.y(), pos.z()) ;

        cap.connect() ;
        cmove.actionStarted.connect(boost::bind(&RotateAndGrab::startCapture, this)) ;
        cmove.actionCompleted.connect(boost::bind(&RotateAndGrab::stopCapture, this)) ;
    }

    void rotate(double theta)
    {
        rotateGripper(cmove, arm, theta) ;

        while (!captureStoped ) ;
    }

    void startCapture()
    {
        captureStoped = false ;
        boost::thread capture_thread(boost::bind(&RotateAndGrab::doCapture, this)) ;
    }

    void doCapture()
    {
        bool _stoped = false ;

        tf::TransformListener listener(ros::Duration(1.0));

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

                if ( cap.grab(clr, depth, ts, cm) )
                {

                    tf::StampedTransform transform;

                    Eigen::Affine3d pose ;

                    try {
                        listener.waitForTransform(camera + "_rgb_optical_frame", arm + "_ee", ts, ros::Duration(1) );
                        listener.lookupTransform(camera + "_rgb_optical_frame", arm + "_ee", ts, transform);
                        tf::TransformTFToEigen(transform, pose);

                        process(clr, depth, cm, ts, pose) ;

                    } catch (tf::TransformException ex) {
                        ROS_INFO("%s",ex.what());
                    }


                }

            }

        } while ( !_stoped ) ;

    }

    // set the robot speed to 1.3 for ~200 pics
    void stopCapture()
    {
        {
            boost::mutex::scoped_lock lock_(mutex) ;
            captureStoped = true ;
        }

    }

    virtual void process(const cv::Mat &clr, const cv::Mat &depth, const image_geometry::PinholeCameraModel cm, const ros::Time &ts, Affine3d &tip_pose_in_camera_frame)
    {
        cout << counter++ << endl ;

        ros::Duration(0.1).sleep() ;
        cv::imwrite(str(boost::format("/tmp/cap_%d.png") % counter), clr) ;
    }

    MoveRobot cmove ;
    camera_helpers::OpenNICaptureRGBD cap ;
    string camera, arm ;
    bool captureStoped ;
    boost::mutex mutex ;

    int counter ;
};



int main(int argc, char **argv) {
    ros::init(argc, argv, "move_rotate_grab");
    ros::NodeHandle nh;

    MoveRobot cmove ;
    cmove.setServoMode(false);

  // moveHome() ;
    moveGripperPointingDown(cmove, "r1", 0, -0.7, 1.5) ;

    RotateAndGrab rg("xtion3", "r1") ;

    rg.init(Vector3d(0, -0.7, 1.5)) ;

    rg.rotate(M_PI) ;
    rg.rotate(M_PI) ;

    ros::spin() ;

}
