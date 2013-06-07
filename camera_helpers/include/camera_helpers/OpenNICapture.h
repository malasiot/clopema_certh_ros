#ifndef __OPENNI_CAPTURE_H__
#define __OPENNI_CAPTURE_H__

#include <string>
#include <sensor_msgs/Image.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <sensor_msgs/PointCloud2.h>
#include <boost/thread.hpp>
#include <cv.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace camera_helpers {

// Helper classes for asynchronous capture of OpenNI camera frame
// Consuming the openni.launch associated topics

// capture an RGB abd Depth image pair

class OpenNICaptureRGBD {

typedef void (*ConnectCallback)(OpenNICaptureRGBD *)  ;

public:

	// Create a grabber for the specified camera prefix e.g. xtion2
    OpenNICaptureRGBD(const std::string &prefix) ;
	
    // Connect to the camera. When the camera is ready the callback will be called. With the callback call grab as many times as you like and when finished call disconnect()
    void connect(ConnectCallback cb) ;
	
	// Grab images
    bool grab(cv::Mat &clr, cv::Mat &depth, ros::Time &ts) ;
	
	// disconnect from the camera
    void disconnect() ;
	
private:

    void input_callback(const sensor_msgs::ImageConstPtr& rgb, const sensor_msgs::ImageConstPtr& depth) ;
    void connectCb(ConnectCallback cb) ;

    std::string prefix ;

    ros::NodeHandle nh;
    sensor_msgs::ImageConstPtr tmp_rgb, tmp_depth ;
    message_filters::Subscriber<sensor_msgs::Image> rgb_sub, depth_sub ;
    message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> > sync ;

    boost::mutex image_lock ;

} ;

// capture a point cloud with or without rgb data

class OpenNICapturePointCloud {

    typedef void (*ConnectCallback)(OpenNICapturePointCloud *)  ;
public:

    // Create a grabber for the specified camera prefix e.g. xtion2
    OpenNICapturePointCloud(const std::string &prefix) ;

    // Connect to the camera
    void connect(ConnectCallback cb) ;

    // Grab images
    bool grab(pcl::PointCloud<pcl::PointXYZ> &cloud, ros::Time &ts) ;
    bool grab(pcl::PointCloud<pcl::PointXYZRGB> &cloud, ros::Time &ts) ;

    // disconnect from the camera
    void disconnect() ;

private:

    void input_callback(const sensor_msgs::PointCloud2ConstPtr &cloud) ;
    void connectCb(ConnectCallback cb) ;

    std::string prefix ;

    ros::NodeHandle nh;
    sensor_msgs::PointCloud2ConstPtr tmp_cloud ;

    message_filters::Subscriber<sensor_msgs::PointCloud2> cloud_sub ;

    boost::mutex cloud_lock ;

} ;



} // namespace camera_helpers



#endif
