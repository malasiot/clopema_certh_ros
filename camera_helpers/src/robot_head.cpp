#include <camera_helpers/RobotHead.h>
#include <ros/ros.h>
#include <cv.h>

#include <RH_cameras/CamerasSync.h>
#include <RH_cameras/CameraInfo.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <ros/callback_queue.h>

#include <boost/thread/thread.hpp>

namespace enc = sensor_msgs::image_encodings;

using namespace std ;
using namespace sensor_msgs;
using namespace message_filters;

namespace camera_helpers {

class CaptureThread
{
public:

    CaptureThread(): sync(sync_policies::ApproximateTime<Image, Image, RH_cameras::CameraInfo, RH_cameras::CameraInfo>(10), left_image_sub_, right_image_sub_, left_camera_info_sub_, right_camera_info_sub_) {


        nh_.setCallbackQueue(&queue);

        left_image_sub_.subscribe(nh_, "/RH/left_camera/image", 1) ;
        right_image_sub_.subscribe(nh_, "/RH/right_camera/image", 1) ;
        left_camera_info_sub_.subscribe(nh_, "/RH/left_camera/camera_info", 1) ;
        right_camera_info_sub_.subscribe(nh_, "/RH/right_camera/camera_info", 1) ;

        sync.registerCallback(boost::bind(&CaptureThread::captureCallback, this, _1, _2, _3, _4)) ;

        dataReady = false ;

    }

    void captureCallback(const ImageConstPtr& left_image_, const ImageConstPtr& right_image_,
                         const RH_cameras::CameraInfoConstPtr &left_cam_info_,
                         const RH_cameras::CameraInfoConstPtr &right_cam_info_)
    {
        cv_bridge::CvImagePtr cv_ptrL, cv_ptrR;
        try
        {
            cv_ptrL = cv_bridge::toCvCopy(left_image_, enc::RGB8);
            cv_ptrR = cv_bridge::toCvCopy(right_image_, enc::RGB8);

            leftIm = cv_ptrL->image ;
            rightIm = cv_ptrR->image ;
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("Could not convert from '%s' to 'rgb8'.", left_image_->encoding.c_str());
            return;
        }

        // Translate camera info (TODO)
        leftInfo.reset(new sensor_msgs::CameraInfo) ;
        leftInfo->width = left_cam_info_->width ;
        leftInfo->height = left_cam_info_->height ;

        rightInfo.reset(new sensor_msgs::CameraInfo) ;
        rightInfo->width = right_cam_info_->width ;
        rightInfo->height = right_cam_info_->height ;


        dataReady = true ;
    }

    void run()
    {
        while ( nh_.ok() && !dataReady )
        {
            queue.callAvailable(ros::WallDuration(0.033)) ;

        }

    }

    ~CaptureThread() {

        left_image_sub_.unsubscribe();
        right_image_sub_.unsubscribe() ;
        left_camera_info_sub_.unsubscribe() ;
        right_camera_info_sub_.unsubscribe() ;
    }

    cv::Mat leftIm, rightIm ;
    sensor_msgs::CameraInfoPtr leftInfo, rightInfo ;

private:

    ros::NodeHandle nh_ ;
    message_filters::Subscriber<sensor_msgs::Image> left_image_sub_, right_image_sub_ ;
    message_filters::Subscriber<RH_cameras::CameraInfo> left_camera_info_sub_, right_camera_info_sub_ ;
    message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, RH_cameras::CameraInfo, RH_cameras::CameraInfo > > sync ;

    ros::CallbackQueue queue ;
    bool dataReady ;

};



bool grabRHImages(cv::Mat &left, cv::Mat &right, image_geometry::StereoCameraModel &cm, bool preview, unsigned int wait)
{
    ros::NodeHandle nh ;

    // start the capture thread

    CaptureThread cap ;

    boost::thread t(boost::bind(&CaptureThread::run, &cap)) ;

    // publish the acqusition command

    ros::Publisher acq_pub ;
    acq_pub = nh.advertise<RH_cameras::CamerasSync>("/RH/cmd/acquire", 1) ;

    RH_cameras::CamerasSync msgStr ;

    msgStr.data = ( preview ) ? "preview" : "full" ;
    msgStr.timeStamp = ros::Time::now() ;

    acq_pub.publish(msgStr) ;

    // start spinner to allow the connection callback to be called

    ros::AsyncSpinner spinner(4) ;
    spinner.start() ;

    // wait until the images are received

    if ( wait == 0 ) {
        t.join() ;
    }
    else
    {
        if ( !t.timed_join(boost::posix_time::milliseconds(wait) ) )  return false ;
    }

    left = cap.leftIm ;
    right = cap.rightIm ;

   // cm.fromCameraInfo(cap.leftInfo, cap.rightInfo) ;

    return true ;
}






























}
