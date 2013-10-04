#include <OpenNI.h>
#include <opencv2/opencv.hpp>
#include <vector>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <camera_info_manager/camera_info_manager.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/distortion_models.h>
#include <boost/thread.hpp>

#include <signal.h>

using namespace std;
using namespace openni;

class OpenNI2Driver {
public:
    OpenNI2Driver(): it(ros::NodeHandle("~")) {}
    ~OpenNI2Driver() { stop() ; }

    bool init(const string &device_id) ;
    bool start() ;
    bool stop() ;


private:

    void run() ;
    string findDevice(const string &device_id) ;

    sensor_msgs::CameraInfoPtr getDefaultCameraInfo(int width, int height, double f) ;

    ros::NodeHandle nh ;
    image_transport::ImageTransport it ;

    image_transport::Publisher image_pub_depth ;
    image_transport::Publisher image_pub_rgb ;
    ros::Publisher pub_depth_camera_info ;
    ros::Publisher pub_rgb_camera_info ;
    boost::shared_ptr<camera_info_manager::CameraInfoManager> rgb_info_manager_ ;
    boost::shared_ptr<camera_info_manager::CameraInfoManager> depth_info_manager_ ;

    openni::VideoStream depthStream;
    openni::VideoStream rgbStream;
    openni::Device device;
    openni::VideoStream** streams ;

    boost::thread capture_thread ;
};

bool OpenNI2Driver::start()
{
    if ( rgbStream.start() != STATUS_OK ) return false ;
    if ( depthStream.start() != STATUS_OK ) return false ;
    return true ;
}

bool OpenNI2Driver::stop()
{
    rgbStream.stop() ;
    depthStream.stop() ;
    return true ;
}

string OpenNI2Driver::findDevice(const string &device_id_)
{
    Array<DeviceInfo> devs ;
    OpenNI::enumerateDevices(&devs) ;

    // look for '#<number>' format
    if (device_id_.size() > 1 && device_id_[0] == '#')
    {
        std::istringstream device_number_str(device_id_.substr(1));
        int device_number;

        device_number_str >> device_number;

        int device_index = device_number - 1; // #1 refers to first device
        if (device_index >= devs.getSize() || device_index < 0)
        {
            ROS_ERROR(
            "Invalid device number %i, there are %zu devices connected.",
            device_number, devs.getSize());

            return string() ;
        }
        else
        {
            return devs[device_index].getUri() ;
        }
    }

    // look for '<bus>@<number>' format
    // <bus> is usb bus id, typically start at 1
    // <number> is the device number, for consistency with openni_camera, these start at 1
    // although 0 specifies "any device on this bus"
    else if (device_id_.size() > 1 && device_id_.find('@') != std::string::npos)
    {
        // get index of @ character
        size_t index = device_id_.find('@');
        if (index <= 0)
        {
            ROS_ERROR(
                "%s is not a valid device URI, you must give the bus number before the @.",
                device_id_.c_str());
            return string() ;
        }

        if (index >= device_id_.size() - 1)
        {
            ROS_ERROR(
                "%s is not a valid device URI, you must give a number after the @, specify 0 for first device",
                device_id_.c_str());
        }

        // pull out device number on bus
        std::istringstream device_number_str(device_id_.substr(index+1));
        int device_number;
        device_number_str >> device_number;

        // reorder to @<bus>
        std::string bus = device_id_.substr(0, index);
        bus.insert(0, "@");
        bus.append("/") ;
        bus.append(device_number_str.str()) ;

        for(int i=0 ; i<devs.getSize() ; i++ )
        {
            const DeviceInfo &info = devs[i] ;

            string uri = info.getUri() ;

            if (uri.find(bus) != std::string::npos)
                return uri ;
        }

        ROS_ERROR("Device not found %s", device_id_.c_str());
        return string() ;
    }
    // everything else is treated as device_URI directly
    else
    {
        return device_id_;
    }
}

bool OpenNI2Driver::init(const string &device_id)
{
    Status initStatus = OpenNI::initialize();

    if (initStatus != STATUS_OK)
    {
        ROS_ERROR("Device could not be initialized because %s", OpenNI::getExtendedError());
        return false ;
    }

    string deviceUri = findDevice(device_id) ;

    if ( deviceUri.empty() ) return false ;

    Status openStatus = device.open(deviceUri.c_str());

    if ( openStatus != STATUS_OK ) {
        ROS_ERROR("Device could not be opened because %s", OpenNI::getExtendedError());
        return false ;
    }

    if ( device.setDepthColorSyncEnabled(true) != STATUS_OK )
    {
        ROS_ERROR("Could not enable color and depth synchronization. Reason: %s",  OpenNI::getExtendedError());
    }



    // Initialize Publisher for depth and rgb image and advertise
    image_pub_depth = it.advertise("depth/image_raw", 1);
    image_pub_rgb = it.advertise("rgb/image_raw", 1);
    pub_depth_camera_info = nh.advertise<sensor_msgs::CameraInfo>("depth/camera_info", 1);
    pub_rgb_camera_info = nh.advertise<sensor_msgs::CameraInfo>("rgb/camera_info", 1);

    string rgb_info_url, depth_info_url;

    nh.param("rgb_camera_info_url", rgb_info_url, string());
    nh.param("depth_camera_info_url", depth_info_url, string());

    // Load the saved calibrations, if they exist
    rgb_info_manager_ =  boost::make_shared<camera_info_manager::CameraInfoManager>(nh, "rgb", rgb_info_url);
    depth_info_manager_  = boost::make_shared<camera_info_manager::CameraInfoManager>(nh,  "depth",  depth_info_url);

    if (!rgb_info_manager_->isCalibrated())
        ROS_WARN("Using default parameters for RGB camera calibration.");
    if (!depth_info_manager_->isCalibrated())
        ROS_WARN("Using default parameters for IR camera calibration.");

    depthStream.create(device, SENSOR_DEPTH);
    rgbStream.create(device, SENSOR_COLOR);

    streams = new openni::VideoStream*[2];
    streams[0] = &depthStream;
    streams[1] = &rgbStream;

    if ( device.isImageRegistrationModeSupported(IMAGE_REGISTRATION_DEPTH_TO_COLOR) )
    {
        ImageRegistrationMode mode ;
        if ( ( mode = device.getImageRegistrationMode() ) != IMAGE_REGISTRATION_DEPTH_TO_COLOR )
        {
            if ( device.setImageRegistrationMode(IMAGE_REGISTRATION_DEPTH_TO_COLOR) != STATUS_OK )
            {
                ROS_ERROR("Could not set image registration. Reason: %s",  OpenNI::getExtendedError());
            }
        }
    }

    capture_thread = boost::thread(boost::bind(&OpenNI2Driver::run, this)) ;

    return true ;

}

void OpenNI2Driver::run()
{
    cv::Mat depthImage;
    cv::Mat rgbImage;

    cv_bridge::CvImagePtr cv_ptr_depth(new cv_bridge::CvImage);
    cv_bridge::CvImagePtr cv_ptr_rgb(new cv_bridge::CvImage);

    while (ros::ok())
    {
        int changedIndex;
        OpenNI::waitForAnyStream( streams, 2, &changedIndex );
        // capture time as close to recording as possible
        ros::Time time = ros::Time::now();

        switch (changedIndex)
        {
            case 0:
            {
                openni::VideoFrameRef depthFrame;
                depthStream.readFrame( &depthFrame);

                if ( depthFrame.isValid() )
                {
                    depthImage = cv::Mat(depthStream.getVideoMode().getResolutionY(),
                    depthStream.getVideoMode().getResolutionX(), CV_16U, (char*)depthFrame.getData() );

                    // convert cv::Mat into cv_bridge image
                    cv_ptr_depth->image = depthImage;
                    cv_ptr_depth->encoding = "16UC1";
                    cv_ptr_depth->header.frame_id = "/openni2_depth_frame";
                    cv_ptr_depth->header.stamp = time;
                    image_pub_depth.publish(cv_ptr_depth->toImageMsg());

                    sensor_msgs::CameraInfoPtr info = getDefaultCameraInfo(640, 480, 570.3422241210938);
                    info->K[2] -= 5; // cx
                    info->K[5] -= 4; // cy
                    info->P[2] -= 5; // cx
                    info->P[6] -= 4; // cy
                    // Fill in header
                    info->header.stamp    = time;
                    info->header.frame_id = "/openni2_depth_frame";
                    pub_depth_camera_info.publish(info);
                }
            }
            break;

            case 1:
            {
                openni::VideoFrameRef rgbFrame;
                rgbStream.readFrame( &rgbFrame);
                if ( rgbFrame.isValid() )
                {
                    rgbImage = cv::Mat(rgbStream.getVideoMode().getResolutionY(),
                        rgbStream.getVideoMode().getResolutionX(), CV_8UC3, (char*)rgbFrame.getData() );

                    // convert cv::Mat into cv_bridge image
                    cv_ptr_rgb->image = rgbImage;
                    cv_ptr_rgb->encoding = "rgb8";
                    cv_ptr_rgb->header.frame_id = "/openni2_rgb_frame";
                    cv_ptr_rgb->header.stamp = time;
                    image_pub_rgb.publish(cv_ptr_rgb->toImageMsg());

                    sensor_msgs::CameraInfoPtr info = getDefaultCameraInfo(640, 480, 525);
                    // Fill in header
                    info->header.stamp    = time;
                    info->header.frame_id = "/openni2_rgb_frame";
                    pub_rgb_camera_info.publish(info);
                }
            }
            break;

            default:
                ROS_WARN("Index %i is neither a depth nor a rgb image stream", changedIndex);
        }
        ros::spinOnce();
    }

}


// the following function was copied from openni_camera


sensor_msgs::CameraInfoPtr OpenNI2Driver::getDefaultCameraInfo(int width, int height, double f)
{
  sensor_msgs::CameraInfoPtr info = boost::make_shared<sensor_msgs::CameraInfo>();

  info->width  = width;
  info->height = height;

  // No distortion
  info->D.resize(5, 0.0);
  info->distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;

  // Simple camera matrix: square pixels (fx = fy), principal point at center
  info->K.assign(0.0);
  info->K[0] = info->K[4] = f;
  info->K[2] = (width / 2) - 0.5;
  // Aspect ratio for the camera center on Kinect (and other devices?) is 4/3
  // This formula keeps the principal point the same in VGA and SXGA modes
  info->K[5] = (width * (3./8.)) - 0.5;
  info->K[8] = 1.0;

  // No separate rectified image plane, so R = I
  info->R.assign(0.0);
  info->R[0] = info->R[4] = info->R[8] = 1.0;

  // Then P=K(I|0) = (K|0)
  info->P.assign(0.0);
  info->P[0]  = info->P[5] = f; // fx, fy
  info->P[2]  = info->K[2];     // cx
  info->P[6]  = info->K[5];     // cy
  info->P[10] = 1.0;

  return info;
}

boost::shared_ptr<OpenNI2Driver> driver ;

void sig_handler(int sig) {

    driver->stop() ;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "openni2_camera");
    ros::NodeHandle nh;

    signal(SIGSEGV, sig_handler);

    driver.reset(new OpenNI2Driver) ;

    if ( driver->init("2@11") )
    {
        //driver->stop() ;
        driver->start() ;

        ros::spin() ;
    }


    return 0;
}
