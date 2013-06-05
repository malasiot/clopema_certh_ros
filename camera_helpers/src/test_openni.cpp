#include <ros/ros.h>

#include "camera_helpers/OpenniCapture.h"

// OpenCV includes
#include <opencv2/highgui/highgui.hpp>
using namespace std ;

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/ros/conversions.h>

namespace enc = sensor_msgs::image_encodings;

using namespace camera_helpers;

class OpenniGrabber {

public:
    OpenniGrabber()
    {

         client = nh.serviceClient<OpenniCapture>("capture");
    }

    bool grab(cv::Mat &rgb, cv::Mat &depth, pcl::PointCloud<pcl::PointXYZ> &pc)
    {
        OpenniCapture srv;

        if (client.call(srv))
        {
            if ( srv.response.success )
            {

                // Save stamps for info
                ros::Time stamp_rgb = srv.response.rgb.header.stamp;
                ros::Time stamp_depth = srv.response.depth.header.stamp;
                ros::Time stamp_cloud = srv.response.cloud.header.stamp;

                cv_bridge::CvImagePtr rgb_ = cv_bridge::toCvCopy(srv.response.rgb, enc::BGR8);
                cv_bridge::CvImagePtr depth_ = cv_bridge::toCvCopy(srv.response.depth, "");

                pcl::fromROSMsg(srv.response.cloud, pc) ;

                rgb = rgb_->image ;
                depth = depth_->image ;

                return true ;
            }
            else return false ;
         }
        else return false ;

    }

private:

    ros::NodeHandle nh ;
    ros::ServiceClient client ;

};


int main(int argc, char** argv) {
    ros::init(argc, argv, "openni_capture_example");
    ros::NodeHandle nh ;

    OpenniGrabber grabber ;

    cv::Mat rgb, depth ;
    pcl::PointCloud<pcl::PointXYZ> pc ;



    if ( grabber.grab(rgb, depth, pc) )
    {
        cv::imwrite("/home/clopema/tmp/rgb.png", rgb) ;
        cv::imwrite("/home/clopema/tmp/depth.png", depth) ;


        pcl::io::savePCDFile("/home/clopema/tmp/cloud.pcd", pc,  true) ;
     }
     else
     {
        ROS_ERROR("Failed to call service");
        return 1;
     }

    return 0;
}
