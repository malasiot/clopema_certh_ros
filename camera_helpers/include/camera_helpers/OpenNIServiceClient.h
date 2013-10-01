#ifndef __OPENNI_SERVICE_CLIENT_H__
#define __OPENNI_SERVICE_CLIENT_H__

#include <ros/ros.h>
#include <string>
#include <cv.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <image_geometry/pinhole_camera_model.h>

namespace camera_helpers {

namespace openni {

    bool connect(const std::string &camera) ;

    bool grab(const std::string &camera, cv::Mat &clr, cv::Mat &depth, ros::Time &ts, image_geometry::PinholeCameraModel &cm) ;
    bool grab(const std::string &camera, pcl::PointCloud<pcl::PointXYZ> &cloud, ros::Time &ts) ;
    bool grab(const std::string &camera, pcl::PointCloud<pcl::PointXYZRGB> &cloud, ros::Time &ts) ;
    bool grab(const std::string &camera, cv::Mat &clr, cv::Mat &depth, pcl::PointCloud<pcl::PointXYZ> &cloud, ros::Time &ts, image_geometry::PinholeCameraModel &cm) ;

    bool disconnect(const std::string &camera) ;
} // namespace openni
} //namespace camera_helpers


#endif
