#ifndef __OPENNI_CAPTURE_H__
#define __OPENNI_CAPTURE_H__

#include <ros/ros.h>
#include <string>
#include <cv.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <boost/function.hpp>
#include <image_geometry/pinhole_camera_model.h>

namespace camera_helpers {

// Helper classes for asynchronous capture of OpenNI camera frame
// Consuming the openni.launch associated topics

class OpenNICaptureBase {
public:

    // Create a grabber for the specified camera prefix e.g. xtion2
    virtual ~OpenNICaptureBase() ;

    // Connect to the camera waiting for data to become available.
    bool connect(ros::Duration timeout = ros::Duration(-1)) ;

    // non blocking version that signals a callback when data is ready
    void connect(boost::function<void ()> cb, ros::Duration timeout = ros::Duration(-1)) ;

    // disconnect from the camera
    void disconnect() ;

    bool isConnected() const ;

protected:

    class OpenNICaptureImpl *impl_ ;

};

// capture an RGB abd Depth image pair (uses registered frame)

class OpenNICaptureRGBD: public OpenNICaptureBase {

public:

	// Create a grabber for the specified camera prefix e.g. xtion2
    OpenNICaptureRGBD(const std::string &prefix) ;

    // Grab images. Also get the timestamp the depth image and the pinhole camera model of the (registered) depth frame

    bool grab(cv::Mat &clr, cv::Mat &depth, ros::Time &ts, image_geometry::PinholeCameraModel &cm) ;

} ;

// capture a point cloud with or without rgb data

class OpenNICapturePointCloud: public OpenNICaptureBase {

public:

    // Create a grabber for the specified camera prefix e.g. xtion2
    OpenNICapturePointCloud(const std::string &prefix) ;

    // Grab images
    bool grab(pcl::PointCloud<pcl::PointXYZ> &cloud, ros::Time &ts) ;
    bool grab(pcl::PointCloud<pcl::PointXYZRGB> &cloud, ros::Time &ts) ;

} ;

// Capture RGBD and Pointcloud

class OpenNICaptureAll: public OpenNICaptureBase {

public:

    // Create a grabber for the specified camera prefix e.g. xtion2
    OpenNICaptureAll(const std::string &prefix) ;

    // Grab images
    bool grab(cv::Mat &clr, cv::Mat &depth, pcl::PointCloud<pcl::PointXYZ> &cloud, ros::Time &ts, image_geometry::PinholeCameraModel &cm) ;

} ;


} // namespace camera_helpers



#endif
