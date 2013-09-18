#ifndef __ROTATE_AND_GRAB_H__
#define __ROTATE_AND_GRAB_H__

#include <Eigen/Geometry>
#include <string>
#include <robot_helpers/Robot.h>
#include <camera_helpers/OpenNICapture.h>

class RotateAndGrab {
public:

    RotateAndGrab(const std::string &camera_, const std::string &arm_) ;

    void init(const Eigen::Vector3d &pos) ;

    void rotate(double theta) ;

protected:

    virtual void process(const cv::Mat &clr, const cv::Mat &depth, const image_geometry::PinholeCameraModel cm,
                         const ros::Time &ts, Eigen::Affine3d &tip_pose_in_camera_frame) = 0;

private:
    void startCapture() ;
    void doCapture() ;
    void stopCapture() ;

    robot_helpers::MoveRobot cmove ;
    camera_helpers::OpenNICaptureRGBD cap ;
    std::string camera, arm ;
    bool captureStoped ;
    boost::mutex mutex ;
};


#endif
