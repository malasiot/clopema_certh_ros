#include "RotateAndGrab.h"

#include <robot_helpers/Utils.h>
#include <robot_helpers/Geometry.h>

#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>


using namespace Eigen ;
using namespace std ;
using namespace robot_helpers ;
using namespace camera_helpers ;

RotateAndGrab::RotateAndGrab(const string &camera_, const string &arm_): camera(camera_), arm(arm_) {
    cmove.setServoMode(false);
}

void RotateAndGrab::init(const Vector3d &pos)
{
    moveGripper(cmove, arm, pos, lookAt(Vector3d(0, 0, -1), 0)) ;

    openni::connect(camera) ;

    cmove.actionStarted.connect(boost::bind(&RotateAndGrab::startCapture, this)) ;
    cmove.actionCompleted.connect(boost::bind(&RotateAndGrab::stopCapture, this)) ;
}

void RotateAndGrab::rotate(double theta)
{
    cmove.setServoMode(false);
    rotateGripper(cmove, arm, theta) ;

    while (!captureStoped ) ;

    capture_thread.join() ;

    openni::disconnect(camera) ;
}

void RotateAndGrab::startCapture()
{
    captureStoped = false ;
    capture_thread = boost::thread(boost::bind(&RotateAndGrab::doCapture, this)) ;
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

            if ( openni::grab(camera, clr, depth, ts, cm) )
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

