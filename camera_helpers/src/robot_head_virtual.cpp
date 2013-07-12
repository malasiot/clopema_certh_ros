#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <highgui.h>
#include <cv.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/String.h>
#include <RH_cameras/CamerasSync.h>
#include <RH_cameras/CameraInfo.h>



using namespace std;

ros::Publisher pub_joint_state_ ;

image_transport::Publisher image_pub_left_, image_pub_right_;
ros::Subscriber sub_ ;
ros::Publisher camera_info_left_pub_, camera_info_right_pub_ ;

void publishJointState()
{

    sensor_msgs::JointState rh_state_ ;

    rh_state_.name.push_back("left_to_pan");
    rh_state_.name.push_back("right_to_pan");
    rh_state_.name.push_back("left_to_tilt");
    rh_state_.name.push_back("right_to_tilt");

    int nJoints = rh_state_.name.size() ;

    rh_state_.position.resize(nJoints);
    rh_state_.velocity.resize(nJoints);
    rh_state_.effort.resize(nJoints);

    for(int i=0 ; i<nJoints ; i++)
    {
        rh_state_.position[i] = 0.0 ;
        rh_state_.velocity[i] = 0.0 ;
        rh_state_.effort[i] = 0.0 ;
    }

    rh_state_.header.stamp = ros::Time::now() ;

    pub_joint_state_.publish(rh_state_) ;


}

void publishImage(const cv::Mat &im, const string &cameraName, bool preview, const ros::Time &timeStamp, image_transport::Publisher pub_, ros::Publisher &camera_info_pub_)
{
    ROS_INFO("Capturing Image from %s", cameraName.c_str() );

    RH_cameras::CameraInfo info;


    try
    {
        cv::Mat rgb;	// image from camera

        cv_bridge::CvImage cvi;
        ros::Time time = timeStamp;

        ROS_INFO_STREAM(im.channels() << endl) ;
        cv::cvtColor(im, rgb, CV_BGR2RGB);

        unsigned int w = rgb.cols, h = rgb.rows ;

        if ( preview )
        {
            w /= 4 ; h /= 4 ;
            cv::resize(rgb, rgb, cv::Size(w, h)) ;
        }

        // convert OpenCV image to ROS message
        cvi.header.stamp = time;
        cvi.header.frame_id = cameraName;
        cvi.encoding = "rgb8";
        cvi.image = rgb;

        info.header.stamp = cvi.header.stamp;
        info.header.frame_id = cameraName ;
        info.width = w;
        info.height = h;

        pub_.publish(cvi.toImageMsg());

        camera_info_pub_.publish(info);
    }
    catch(cv_bridge::Exception& e)
    {
        ROS_ERROR("RHcam_node exception: %s", e.what());
        return;
    }


}


void captureImages(const string &pathL, const string &pathR, const RH_cameras::CamerasSync::ConstPtr& msg)
{
    cv::Mat imL = cv::imread(pathL, -1) ;
    cv::Mat imR = cv::imread(pathR, -1) ;

    bool preview = msg->data == "full" ? false : true ;

    publishImage(imL, "camera_left", preview, msg->timeStamp, image_pub_left_, camera_info_left_pub_) ;
    publishImage(imR, "camera_right", preview, msg->timeStamp, image_pub_right_, camera_info_right_pub_) ;

}

int main(int argc, char *argv[])
{

    ros::init(argc, argv, "robot_head_virtual");

    string imPathL ;
    string imPathR ;

    ros::NodeHandle nh_ ;

    nh_.getParam("RH_virtual/imL", imPathL) ;
    nh_.getParam("RH_virtual/imR", imPathR) ;

    pub_joint_state_ = nh_.advertise<sensor_msgs::JointState>("joint_states", 1);

    image_transport::ImageTransport it_(nh_) ;

    image_pub_left_ = it_.advertise("/RH/left_camera/image/", 1);
    image_pub_right_ = it_.advertise("/RH/right_camera/image/", 1);
    camera_info_left_pub_ = nh_.advertise<RH_cameras::CameraInfo>("/RH/left_camera/camera_info",1);
    camera_info_right_pub_ = nh_.advertise<RH_cameras::CameraInfo>("/RH/right_camera/camera_info",1);
    sub_ = nh_.subscribe<RH_cameras::CamerasSync>("/RH/cmd/acquire", 1, boost::bind(&captureImages, imPathL, imPathR, _1));

    ros::spin() ;

    return 0 ;

}
