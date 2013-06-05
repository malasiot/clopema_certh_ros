

#include <iostream>

// ROS Includes
#include "ros/ros.h"
#include "camera_helpers/OpenniCapture.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

// OpenCV includes
#include <opencv2/highgui/highgui.hpp>

// PCL includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

// Definitions
#define WINDOW "capture_view"

// Namaspaces to be used
using namespace camera_helpers;
using namespace sensor_msgs;
using namespace message_filters;

ImageConstPtr tmp_rgb, tmp_depth ;
PointCloud2::ConstPtr tmp_cloud;

bool capture_handle(OpenniCapture::Request &req, OpenniCapture::Response &res)
{
    // Prepare success
    res.success = false;

    // Check whether we have both images
    if (tmp_rgb && tmp_depth && tmp_cloud) {

        // Change response to true
        res.rgb = *tmp_rgb ;
        res.depth = *tmp_depth ;
        res.cloud = *tmp_cloud ;
        res.success = true;
    } else {
        ROS_WARN("No incoming data, nothing to return!");
    }

    return true;
}

void input_callback(const ImageConstPtr& rgb, const ImageConstPtr& depth, const PointCloud2::ConstPtr cloud)
{
    // Store current images
    tmp_rgb = rgb ;
    tmp_depth = depth ;
    tmp_cloud = cloud;

}

int main(int argc, char **argv)
{
    // Initialize node
    ros::init(argc, argv, "openni_capture");
    ros::NodeHandle nh;

    // Advertise service
    ros::ServiceServer service = nh.advertiseService("capture", capture_handle);

    // Subscribe to rgb and depth streams
    Subscriber<Image> rgb_sub(nh, "rgb", 1);
    Subscriber<Image> depth_sub(nh, "depth", 1);
    Subscriber<PointCloud2> cloud_sub(nh, "cloud", 1);

    Synchronizer<sync_policies::ApproximateTime<Image, Image, PointCloud2> > sync(sync_policies::ApproximateTime<Image, Image, PointCloud2>(10), rgb_sub, depth_sub, cloud_sub);
    sync.registerCallback(boost::bind(&input_callback, _1, _2, _3));

    // Spin ros node
    ros::spin();

    return 0;
}
