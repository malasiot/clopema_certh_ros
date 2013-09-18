#include "RotateAndGrab.h"

#include <robot_helpers/Utils.h>

#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>


using namespace Eigen ;
using namespace std ;
using namespace robot_helpers ;

RotateAndGrab::RotateAndGrab(const string &camera_, const string &arm_): cap(camera_), camera(camera_), arm(arm_) {
    cmove.setServoMode(false);
}

void RotateAndGrab::init(const Vector3d &pos)
{
    moveGripperPointingDown(cmove, arm, pos.x(), pos.y(), pos.z()) ;

    cap.connect() ;
    cmove.actionStarted.connect(boost::bind(&RotateAndGrab::startCapture, this)) ;
    cmove.actionCompleted.connect(boost::bind(&RotateAndGrab::stopCapture, this)) ;
}

void RotateAndGrab::rotate(double theta)
{
    rotateGripper(cmove, arm, theta) ;

    while (!captureStoped ) ;
}

void RotateAndGrab::startCapture()
{
    captureStoped = false ;
    boost::thread capture_thread(boost::bind(&RotateAndGrab::doCapture, this)) ;
}

void RotateAndGrab::doCapture()
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

void RotateAndGrab::stopCapture()
{
    boost::mutex::scoped_lock lock_(mutex) ;
    captureStoped = true ;
}

