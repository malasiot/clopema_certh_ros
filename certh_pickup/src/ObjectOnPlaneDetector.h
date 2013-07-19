#ifndef __OBJECT_ON_PLANE_DETECTOR_H__
#define __OBJECT_ON_PLANE_DETECTOR_H__

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <Eigen/Geometry>

#include <cv.h>

typedef pcl::PointXYZRGB PointType ;
typedef pcl::PointCloud<PointType> CloudType ;

class ObjectOnPlaneDetector {

public:

    ObjectOnPlaneDetector(const CloudType &cloud) ;
    ObjectOnPlaneDetector(const cv::Mat &clr_im, const cv::Mat &depth_im,
                           double fx, double fy, double cx, double cy)  ;


    // find object plane. this is the plane with the higher number of inliers to the plane model
    bool findPlane(Eigen::Vector3d &n, double &d, cv::Mat &inliers) ;

    // find an image mask that designates plane pixels by discarding pixels beyond and above the specified plane
    // Mask filtering is performed to obtain a single cluster

    cv::Mat findObjectMask(const Eigen::Vector3d &n, double d, double thresh, cv::Mat &dmap, std::vector<cv::Point> &hull) ;

    cv::Mat getForegroundMask(const cv::Mat &inmask, std::vector<cv::Point> &hull, int minArea) ;

    cv::Mat refineSegmentation(const cv::Mat &clr, const cv::Mat &fgMask, std::vector<cv::Point> &hull) ;


private:

    CloudType cvMatToCloud(const cv::Mat &clr_im, const cv::Mat &depth_im, double fx, double fy, double ox, double oy) ;

private:

    CloudType in_cloud_ ;

};



#endif
