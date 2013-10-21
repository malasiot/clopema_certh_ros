#ifndef __ROTATE_AND_GRAB_H__
#define __ROTATE_AND_GRAB_H__

#include <Eigen/Geometry>
#include <string>
#include <robot_helpers/Robot.h>
#include <camera_helpers/OpenNIServiceClient.h>

class RotateAndGrab {
public:

    RotateAndGrab(const std::string &camera_, const std::string &arm_) ;

    void init(const Eigen::Vector3d &pos) ;

    void rotate(double theta) ;

    struct rotateData {

        int cx ;
        int dataCounter;
        std::vector <pcl::PointCloud<pcl::PointXYZ> > cloud ;
        std::vector < cv::Mat > clr ;
        std::vector < cv::Mat > depth ;
        std::vector < Eigen::Matrix4d > orientations ;

    } ;
protected:

    virtual void process(const pcl::PointCloud<pcl::PointXYZ> &pc, const cv::Mat &clr, const cv::Mat &depth, const image_geometry::PinholeCameraModel &cm,
                         const ros::Time &ts, Eigen::Affine3d &tip_pose_in_camera_frame) = 0;


    void startCapture() ;
    void doCapture() ;
    void stopCapture() ;

    robot_helpers::MoveRobot cmove ;

    std::string camera, arm ;
    bool captureStoped ;
    boost::mutex mutex ;
    boost::thread capture_thread ;
};


#endif
