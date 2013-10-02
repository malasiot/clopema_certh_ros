#include <ros/ros.h>
#include <camera_helpers/OpenNIServiceConnect.h>
#include <camera_helpers/OpenNIServiceGrab.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>
#include <boost/thread/locks.hpp>
#include <pcl/ros/conversions.h>
#include <boost/thread.hpp>


using namespace std ;
using namespace sensor_msgs;
using namespace message_filters;

class OpenNIService {

public:

    OpenNIService(const string &cam_prefix): prefix(cam_prefix), connected(false),
       sync(sync_policies::ApproximateTime<Image, Image, CameraInfo>(10), rgb_sub, depth_sub, camera_sub),
       spinner(4), data_ready(false)
       {
            sync.registerCallback(boost::bind(&OpenNIService::input_callback, this, _1, _2, _3));
            spinner.start() ;
       }

    bool grab(camera_helpers::OpenNIServiceGrab::Request &req, camera_helpers::OpenNIServiceGrab::Response &res )
    {
        if ( !connected ) return false ;

        boost::unique_lock<boost::mutex> lock_ (image_lock) ;

        res.clr = *tmp_rgb ;
        res.depth = *tmp_depth ;
        res.camera = *tmp_camera ;
        res.ts.data = ros::Time::now() ;

        return true ;
    }

    bool connect(camera_helpers::OpenNIServiceConnect::Request &req, camera_helpers::OpenNIServiceConnect::Response &res )
    {
        if ( req.connect )
        {
            if ( !connected ) do_connect() ;
        }
        else
        {
            if ( connected ) disconnect() ;
        }

        res.status = 0 ;
    }


    void do_connect()
    {
        // Subscribe to rgb and depth streams

        rgb_sub.subscribe(nh, "/" + prefix + "/rgb/image_rect_color", 10);
        depth_sub.subscribe(nh, "/" + prefix + "/depth_registered/image_rect_raw", 10);
        camera_sub.subscribe(nh, "/" + prefix + "/depth_registered/camera_info", 10);

        if ( connected ) disconnect() ;

        while ( !data_ready )
        {
            ros::Duration(0.1).sleep() ;
            ros::spinOnce() ;
        }

        connected = true ;

        ROS_INFO("connected to %s", prefix.c_str()) ;

    }

    virtual void disconnect()
    {
        rgb_sub.unsubscribe();
        depth_sub.unsubscribe();
        camera_sub.unsubscribe() ;

        connected = false ;

        ROS_INFO("disconnected from %s", prefix.c_str()) ;
    }

    void input_callback(const ImageConstPtr& rgb, const ImageConstPtr& depth, const CameraInfoConstPtr &camera)
    {
        {
            boost::unique_lock<boost::mutex> lock_ (image_lock) ;

            // Store current images
            tmp_rgb = rgb ;
            tmp_depth = depth ;
            tmp_camera = camera ;

            data_ready = true ;

        }

    }

    string prefix ;
    sensor_msgs::ImageConstPtr tmp_rgb, tmp_depth ;
    sensor_msgs::CameraInfoConstPtr tmp_camera ;
    message_filters::Subscriber<sensor_msgs::Image> rgb_sub, depth_sub ;
    message_filters::Subscriber<sensor_msgs::CameraInfo> camera_sub ;
    message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo> > sync ;
    ros::NodeHandle nh;
    boost::mutex image_lock ;
    bool connected, data_ready ;
    ros::AsyncSpinner spinner ;

};


int main(int argc, char **argv)
{
    if ( argc < 2 ) {
        ROS_ERROR("No camera specified") ;
        return 0 ;
    }
    string camera = argv[1] ;

    ros::init(argc, argv, "openni_service" + camera );



    

    ros::NodeHandle nh("~") ;

    OpenNIService srv(camera) ;

    // Register the service with the master
    ros::ServiceServer grabber = nh.advertiseService(camera + "/grab", &OpenNIService::grab, &srv  );
    ros::ServiceServer connect = nh.advertiseService(camera + "/connect", &OpenNIService::connect, &srv  );

    ros::spin() ;

}
