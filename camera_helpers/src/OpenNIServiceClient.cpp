#include <camera_helpers/OpenNIServiceClient.h>
#include <camera_helpers/OpenNIServiceConnect.h>
#include <camera_helpers/OpenNIServiceGrab.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/ros/conversions.h>

namespace enc = sensor_msgs::image_encodings;

using namespace std ;
using namespace sensor_msgs;


namespace camera_helpers {
namespace openni {


bool doConnect(const std::string &camera, bool con)
{
    ros::NodeHandle nh ;
    ros::ServiceClient client = nh.serviceClient<camera_helpers::OpenNIServiceConnect>(camera + "/openni_service/connect");

    if ( !client.waitForExistence() ) {
        ROS_ERROR("Can't connect to service %s", client.getService().c_str()) ;
        return false ;
    }

    camera_helpers::OpenNIServiceConnect::Request req ;
    camera_helpers::OpenNIServiceConnect::Response res ;

    req.connect = con ;

    if ( !client.call(req, res) )
    {
        ROS_ERROR("Failed to call service %s", client.getService().c_str()) ;
        return false ;
    }

    return ( res.status == 0 ) ? true : false ;

}

bool connect(const std::string &camera)
{
    return doConnect(camera, true) ;
}

bool disconnect(const std::string &camera)
{
    return doConnect(camera, false) ;
}



bool captureRGBD(const string &camera, cv::Mat &rgb, cv::Mat &depth, image_geometry::PinholeCameraModel &cm, ros::Time &ts)
{
    ros::NodeHandle nh ;
    ros::ServiceClient client = nh.serviceClient<camera_helpers::OpenNIServiceGrab>(camera + "/openni_service/grab");

    if ( !client.waitForExistence() ) {
        ROS_ERROR("Can't connect to service %s", client.getService().c_str()) ;
        return false ;
    }

    camera_helpers::OpenNIServiceGrab::Request req ;
    camera_helpers::OpenNIServiceGrab::Response res ;

    if ( !client.call(req, res) )
    {
        ROS_ERROR("Failed to call service %s", client.getService().c_str()) ;
        return false ;
    }

    cv_bridge::CvImagePtr rgb_ = cv_bridge::toCvCopy(res.clr, enc::BGR8);
    cv_bridge::CvImagePtr depth_ = cv_bridge::toCvCopy(res.depth, "");

    rgb = rgb_->image ;
    depth = depth_->image ;
    cm.fromCameraInfo(res.camera) ;

    ts = res.ts.data ;

    if ( rgb.data == NULL || depth.data == NULL ) return false ;
    else return true ;

}

bool grab(const std::string &camera, cv::Mat &clr, cv::Mat &depth, ros::Time &ts, image_geometry::PinholeCameraModel &cm)
{
    return captureRGBD(camera, clr, depth, cm, ts) ;
}



typedef union
{
  struct /*anonymous*/
  {
    unsigned char Blue;
    unsigned char Green;
    unsigned char Red;
    unsigned char Alpha;
  };
  float float_value;
  long long_value;
} RGBValue;

bool grab(const std::string &camera, pcl::PointCloud<pcl::PointXYZRGB> &cloud, ros::Time &ts)
{
    cv::Mat rgb, depth ;
    image_geometry::PinholeCameraModel model_ ;

    if ( !captureRGBD(camera, rgb, depth, model_, ts) ) return false ;

    cloud.width = depth.cols ;
    cloud.height = depth.rows ;
    cloud.is_dense = false ;
    cloud.points.resize(cloud.height * cloud.width) ;

    float center_x = model_.cx();
    float center_y = model_.cy();

    double unit_scaling = 0.001 ;
    float constant_x = unit_scaling / model_.fx();
    float constant_y = unit_scaling / model_.fy();
    float bad_point = std::numeric_limits<float>::quiet_NaN();

    pcl::PointCloud<pcl::PointXYZRGB>::iterator pt_iter = cloud.begin();

    cv::Mat_<ushort> depth_(depth) ;
    cv::Mat_<cv::Vec3b> clr_(rgb) ;

    for(int i=0 ; i<depth.rows ; i++)
        for(int j=0 ; j<depth.cols ; j++)
        {
            pcl::PointXYZRGB & pt = *pt_iter++;
            ushort val = depth_[i][j] ;
            cv::Vec3b cval = clr_[i][j] ;

            if ( val == 0 ) {
               pt.x = pt.y = pt.z = bad_point;
            }
            else
            {
                pt.x = (j - center_x) * val * constant_x;
                pt.y = (i - center_y) * val * constant_y;
                pt.z = val * unit_scaling ;
            }


            // Fill in color
            RGBValue color;
            color.Red   = cval[0] ;
            color.Green = cval[1];
            color.Blue  = cval[2];
            color.Alpha = 0;
            pt.rgb = color.float_value;
        }
}



bool grab(const std::string &camera, cv::Mat &rgb, cv::Mat &depth, pcl::PointCloud<pcl::PointXYZ> &cloud, ros::Time &ts, image_geometry::PinholeCameraModel &model_)
{

    if ( !captureRGBD(camera, rgb, depth, model_, ts) ) return false ;

    cloud.width = depth.cols ;
    cloud.height = depth.rows ;
    cloud.is_dense = false ;
    cloud.points.resize(cloud.height * cloud.width) ;

    float center_x = model_.cx();
    float center_y = model_.cy();

    double unit_scaling = 0.001 ;
    float constant_x = unit_scaling / model_.fx();
    float constant_y = unit_scaling / model_.fy();
    float bad_point = std::numeric_limits<float>::quiet_NaN();

    pcl::PointCloud<pcl::PointXYZ>::iterator pt_iter = cloud.begin();

    cv::Mat_<ushort> depth_(depth) ;

    for(int i=0 ; i<depth.rows ; i++)
        for(int j=0 ; j<depth.cols ; j++)
        {
            pcl::PointXYZ & pt = *pt_iter++;
            ushort val = depth_[i][j] ;

            if ( val == 0 ) {
               pt.x = pt.y = pt.z = bad_point;
               continue;
            }

            pt.x = (j - center_x) * val * constant_x;
            pt.y = (i - center_y) * val * constant_y;
            pt.z = val * unit_scaling ;
        }

    return true ;
}


bool grab(const std::string &camera, pcl::PointCloud<pcl::PointXYZ> &cloud, ros::Time &ts)
{

    cv::Mat rgb, depth ;
    image_geometry::PinholeCameraModel model_ ;

    return grab(camera, rgb, depth, cloud, ts, model_) ;
}


}
}

