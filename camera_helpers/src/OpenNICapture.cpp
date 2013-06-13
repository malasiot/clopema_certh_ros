#include <camera_helpers/OpenNICapture.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <boost/thread/locks.hpp>
#include <pcl/ros/conversions.h>

namespace enc = sensor_msgs::image_encodings;

using namespace std ;
using namespace sensor_msgs;
using namespace message_filters;

namespace camera_helpers {



OpenNICaptureRGBD::OpenNICaptureRGBD(const string &prefix_): prefix(prefix_),
    sync(sync_policies::ApproximateTime<Image, Image>(10), rgb_sub, depth_sub)
{

}

void OpenNICaptureRGBD::input_callback(const ImageConstPtr& rgb, const ImageConstPtr& depth)
{
    boost::unique_lock<boost::mutex> lock_ (image_lock) ;
    // Store current images
    tmp_rgb = rgb ;
    tmp_depth = depth ;
}

void OpenNICaptureRGBD::connectCb(ConnectCallback cb)
{
    assert(cb) ;

    while  (!tmp_rgb || !tmp_depth ) ;

    cb(this) ;

}

void OpenNICaptureRGBD::connect(ConnectCallback cb)
{
    // Subscribe to rgb and depth streams

    rgb_sub.subscribe(nh, "/" + prefix + "/rgb/image_color", 1);
    depth_sub.subscribe(nh, "/" + prefix + "/depth_registered/image_rect", 1);

    sync.registerCallback(boost::bind(&OpenNICaptureRGBD::input_callback, this, _1, _2));

    boost::thread t(boost::bind(&OpenNICaptureRGBD::connectCb, this, cb)) ;
}

void OpenNICaptureRGBD::disconnect()
{
    rgb_sub.unsubscribe();
    depth_sub.unsubscribe();
}

bool OpenNICaptureRGBD::grab(cv::Mat &clr, cv::Mat &depth, ros::Time &t)
{
    boost::unique_lock<boost::mutex> lock_ (image_lock) ;

    if ( !tmp_rgb || !tmp_depth ) return false ;

    cv_bridge::CvImagePtr rgb_ = cv_bridge::toCvCopy(tmp_rgb, enc::BGR8);
    cv_bridge::CvImagePtr depth_ = cv_bridge::toCvCopy(tmp_depth, "");

    clr = rgb_->image ;
    depth = depth_->image ;
    t = tmp_depth->header.stamp ;

    return true ;
}


/////////////////////////////////////////////////////////////////////////////////////////



OpenNICapturePointCloud::OpenNICapturePointCloud(const string &prefix_): prefix(prefix_)
{

}

void OpenNICapturePointCloud::input_callback(const PointCloud2ConstPtr &cloud)
{
    boost::unique_lock<boost::mutex> lock_ (cloud_lock) ;
    // Store current images
    tmp_cloud = cloud ;
}

void OpenNICapturePointCloud::connectCb(ConnectCallback cb)
{
    assert(cb) ;

    while  (!tmp_cloud ) ;

    cb(this) ;

}

void OpenNICapturePointCloud::connect(ConnectCallback cb)
{
    // Subscribe to rgb and depth streams

    cloud_sub.subscribe(nh, "/" + prefix + "/depth_registered/points", 1);
    cloud_sub.registerCallback(boost::bind(&OpenNICapturePointCloud::input_callback, this, _1)) ;

    boost::thread t(boost::bind(&OpenNICapturePointCloud::connectCb, this, cb)) ;
}

void OpenNICapturePointCloud::disconnect()
{
    cloud_sub.unsubscribe();

}

bool OpenNICapturePointCloud::grab(pcl::PointCloud<pcl::PointXYZ> &pc, ros::Time &ts)
{
    boost::unique_lock<boost::mutex> lock_ (cloud_lock) ;

    if ( !tmp_cloud ) return false ;

    pcl::fromROSMsg(*tmp_cloud, pc) ;

    ts = tmp_cloud->header.stamp ;

    return true ;
}


bool OpenNICapturePointCloud::grab(pcl::PointCloud<pcl::PointXYZRGB> &pc, ros::Time &ts)
{
    boost::unique_lock<boost::mutex> lock_ (cloud_lock) ;

    if ( !tmp_cloud ) return false ;

    pcl::fromROSMsg(*tmp_cloud, pc) ;

    ts = tmp_cloud->header.stamp ;

    return true ;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


OpenNICaptureAll::OpenNICaptureAll(const string &prefix_): prefix(prefix_),
    sync(sync_policies::ApproximateTime<Image, Image, PointCloud2>(10), rgb_sub, depth_sub, cloud_sub)
{

}

void OpenNICaptureAll::input_callback(const ImageConstPtr& rgb, const ImageConstPtr& depth, const PointCloud2ConstPtr &cloud)
{
    boost::unique_lock<boost::mutex> lock_ (cloud_lock) ;
    // Store current images
    tmp_rgb = rgb ;
    tmp_depth = depth ;
    tmp_cloud = cloud ;
}

void OpenNICaptureAll::connectCb(ConnectCallback cb)
{
    assert(cb) ;

    while  (!tmp_cloud && !tmp_rgb && !tmp_depth) ;

    cb(this) ;

}

void OpenNICaptureAll::connect(ConnectCallback cb)
{
    // Subscribe to rgb and depth streams

    cloud_sub.subscribe(nh, "/" + prefix + "/depth_registered/points", 1);
    rgb_sub.subscribe(nh, "/" + prefix + "/rgb/image_color", 1);
    depth_sub.subscribe(nh, "/" + prefix + "/depth_registered/image_rect", 1);

    sync.registerCallback(boost::bind(&OpenNICaptureAll::input_callback, this, _1, _2, _3)) ;

    boost::thread t(boost::bind(&OpenNICaptureAll::connectCb, this, cb)) ;
}

void OpenNICaptureAll::disconnect()
{
    cloud_sub.unsubscribe();
    rgb_sub.unsubscribe();
    depth_sub.unsubscribe();

}




bool OpenNICaptureAll::grab(cv::Mat &clr, cv::Mat &depth, pcl::PointCloud<pcl::PointXYZ> &pc, ros::Time &ts)
{
    boost::unique_lock<boost::mutex> lock_ (cloud_lock) ;

    if ( !tmp_cloud ) return false ;

    pcl::fromROSMsg(*tmp_cloud, pc) ;

    if ( !tmp_rgb || !tmp_depth ) return false ;

    cv_bridge::CvImagePtr rgb_ = cv_bridge::toCvCopy(tmp_rgb, enc::BGR8);
    cv_bridge::CvImagePtr depth_ = cv_bridge::toCvCopy(tmp_depth, "");

    clr = rgb_->image ;
    depth = depth_->image ;

    ts = tmp_cloud->header.stamp ;

    return true ;
}

}
