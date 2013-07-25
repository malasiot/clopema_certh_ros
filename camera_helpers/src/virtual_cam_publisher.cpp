#include <ros/ros.h>

#include <highgui.h>
#include <cv.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <cv_bridge/CvBridge.h>
#include <camera_info_manager/camera_info_manager.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/distortion_models.h>

#include <boost/filesystem.hpp>
#include <boost/filesystem/path.hpp>
#include <boost/regex.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/algorithm/string/predicate.hpp>
#include <boost/tokenizer.hpp>

using namespace std;

class MonocularImagePublisher
{

public:
    MonocularImagePublisher(const vector<string> &imagePaths, const string &cam_info_url, const string &camFrameId, double rate = 30)
    {
        ros::NodeHandle nh_("virtual_cam") ;

        bool isCalibrated = false ;

        if ( !cam_info_url.empty() )
        {

            log4cxx::LoggerPtr logger_ccp = log4cxx::Logger::getLogger("ros.camera_calibration_parsers");
            logger_ccp->setLevel(log4cxx::Level::getFatal());

            cam_info_manager_ = boost::make_shared<camera_info_manager::CameraInfoManager>(nh_, "camera", cam_info_url);
            isCalibrated = cam_info_manager_->isCalibrated() ;
        }

        counter_ = 0 ;

        image_transport::ImageTransport it(nh_);

        if ( !isCalibrated )
            pub_im_ = it.advertise("image_raw", 1) ;
        else
            pub_cam_ = it.advertiseCamera("image_raw", 1);

        ros::Rate loop_rate(rate);

        while (nh_.ok()) {

            if ( counter_ == imagePaths.size() ) counter_ = 0 ;

            cv::Mat image = cv::imread(imagePaths[counter_], -1) ;
            IplImage ipl_ = image ;

            if ( image.data == NULL )
            {
                ROS_ERROR("Failed to load image %s\n", imagePaths[counter_].c_str()) ;
                break ;
            }

            counter_++ ;

            sensor_msgs::ImagePtr msg ;

            if ( image.type() == CV_16UC1 )
                sensor_msgs::CvBridge::fromIpltoRosImage(&ipl_, *msg, "passthrough");
            else if ( image.type() == CV_8UC3 )
                sensor_msgs::CvBridge::fromIpltoRosImage(&ipl_, *msg, "bgr8");
            else if ( image.type() == CV_8UC1 )
                sensor_msgs::CvBridge::fromIpltoRosImage(&ipl_, *msg, "mono8");


            if ( !isCalibrated )
                pub_im_.publish(msg);
            else {
                sensor_msgs::CameraInfoPtr info;
                info = boost::make_shared<sensor_msgs::CameraInfo>(cam_info_manager_->getCameraInfo());

                info->header.stamp    = ros::Time::now();
                info->header.frame_id = camFrameId ;

                pub_cam_.publish(msg, info) ;

            }

            ros::spinOnce();
            loop_rate.sleep();
        }


    }

    boost::shared_ptr<camera_info_manager::CameraInfoManager> cam_info_manager_ ;
    image_transport::Publisher pub_im_ ;
    image_transport::CameraPublisher pub_cam_ ;
    int counter_ ;
};


class RGBDImagePublisher
{

public:
    RGBDImagePublisher(const vector<string> &imagePathsRGB, const vector<string> &imagePathsDepth,
                       const string &rgb_cam_info_url, const string &depth_cam_info_url,
                       const string &rgb_cam_frame_id, const string &depth_cam_frame_id,
                       bool registered, double rate = 30)
    {
        ros::NodeHandle nh_("virtual_cam") ;

        ros::NodeHandle rgb_nh(nh_, "rgb");
        ros::NodeHandle depth_nh(nh_, "depth");
        ros::NodeHandle depth_registered_nh(nh_, "depth_registered");

        image_transport::ImageTransport rgb_it(rgb_nh);
        image_transport::ImageTransport depth_it(depth_nh);
        image_transport::ImageTransport depth_registered_it(depth_registered_nh);

        pub_cam_rgb_ = rgb_it.advertiseCamera("image_raw", 1);
        pub_cam_depth_ = depth_it.advertiseCamera("image_raw", 1);

        log4cxx::LoggerPtr logger_ccp = log4cxx::Logger::getLogger("ros.camera_calibration_parsers");
        logger_ccp->setLevel(log4cxx::Level::getFatal());

        cam_info_manager_rgb_ = boost::make_shared<camera_info_manager::CameraInfoManager>(rgb_nh, "rgb", rgb_cam_info_url);
        cam_info_manager_depth_ = boost::make_shared<camera_info_manager::CameraInfoManager>(depth_nh, "depth", depth_cam_info_url);

        if ( registered )
            pub_cam_depth_registered_ = depth_registered_it.advertiseCamera("image_raw", 1);

        counter_ = 0 ;

        ros::Rate loop_rate(rate);

        while (nh_.ok()) {

            if ( counter_ == imagePathsRGB.size() ) counter_ = 0 ;

            cv::Mat imageRGB = cv::imread(imagePathsRGB[counter_], -1) ;
            cv::Mat imageD = cv::imread(imagePathsDepth[counter_], -1) ;
            IplImage ipl_rgb_ = imageRGB, ipl_depth_ = imageD ;

            if ( imageRGB.data == NULL || imageRGB.type() != CV_8UC3 )
            {
                ROS_ERROR("Failed to load image %s\n", imagePathsRGB[counter_].c_str()) ;
                break ;
            }

            if ( imageD.data == NULL || imageD.type() != CV_16UC1 )
            {
                ROS_ERROR("Failed to load image %s\n", imagePathsDepth[counter_].c_str()) ;
                break ;
            }

            counter_++ ;

            sensor_msgs::ImagePtr msg_rgb = sensor_msgs::CvBridge::cvToImgMsg(&ipl_rgb_, "bgr8");
            sensor_msgs::ImagePtr msg_depth = sensor_msgs::CvBridge::cvToImgMsg(&ipl_depth_, "passthrough");

            sensor_msgs::CameraInfoPtr infoRGB, infoDepth ;

            if ( cam_info_manager_rgb_->isCalibrated() )
                infoRGB = boost::make_shared<sensor_msgs::CameraInfo>(cam_info_manager_rgb_->getCameraInfo());
            else
                infoRGB = getDefaultCameraInfo(imageRGB.cols, imageRGB.rows, 525) ;

            infoRGB->header.stamp    = ros::Time::now();
            infoRGB->header.frame_id = rgb_cam_frame_id ;

            pub_cam_rgb_.publish(msg_rgb, infoRGB) ;

            if ( cam_info_manager_depth_->isCalibrated() )
                infoDepth = boost::make_shared<sensor_msgs::CameraInfo>(cam_info_manager_depth_->getCameraInfo());
            else
            {
                const double depth_ir_offset_x_ = 5 ;
                const double depth_ir_offset_y_ = 4 ;

                infoDepth = getDefaultCameraInfo(imageD.cols, imageD.rows, 525) ;
                infoDepth->K[2] -= depth_ir_offset_x_; // cx
                infoDepth->K[5] -= depth_ir_offset_y_; // cy
                infoDepth->P[2] -= depth_ir_offset_x_; // cx
                infoDepth->P[6] -= depth_ir_offset_y_; // cy
            }

            infoDepth->header.stamp    = ros::Time::now();
            infoDepth->header.frame_id = depth_cam_frame_id ;

            if ( registered )
                pub_cam_depth_registered_.publish(msg_depth, infoRGB) ;
            else
                pub_cam_depth_.publish(msg_depth, infoDepth) ;


            ros::spinOnce();
            loop_rate.sleep();
        }


    }

    sensor_msgs::CameraInfoPtr getDefaultCameraInfo(int width, int height, double f) const
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

    boost::shared_ptr<camera_info_manager::CameraInfoManager> cam_info_manager_rgb_, cam_info_manager_depth_ ;
    image_transport::CameraPublisher pub_cam_rgb_ ;
    image_transport::CameraPublisher pub_cam_depth_ ;
    image_transport::CameraPublisher pub_cam_depth_registered_ ;

    int counter_ ;
};

bool parse(const string &fileName, vector<string> *filePaths, int &nChannels, const char *channels)
{
      boost::filesystem::path pathPattern(fileName) ;

      boost::filesystem::path dir = pathPattern.parent_path();
      std::string fileNameRegex = pathPattern.filename().string();

      boost::regex rx("%0([[:digit:]])d") ;

      std::string res =  boost::regex_replace(fileNameRegex, rx, "[[:digit:]]{$1}") ;

      if ( channels ) boost::algorithm::replace_all(res, "%c", std::string("([") + channels + "])");

      boost::algorithm::replace_all(res, ".", "\\.") ;
      boost::algorithm::replace_all(res, "*", ".*") ;

      boost::regex fprx(res) ;

      boost::filesystem::directory_iterator it(dir), end ;

      for( ; it != end ; ++it)
      {
          const boost::filesystem::path &fullPath =  (*it).path() ;

          boost::smatch sm ;

          // test of this is a valid filename

          std::string fileName = fullPath.filename().string() ;

          if ( !boost::regex_match(fullPath.filename().string(), sm, fprx ) ) continue ;
          else
          {
              // found try to match channel

              std::string channelStr = sm[1] ;

              if ( !channelStr.empty() )
              {
                  int channel = channelStr[0] ;

                  int ch = strchr(channels, channel) - channels ;

                  filePaths[ch].push_back(fullPath.string()) ;
              }
              else filePaths[0].push_back(fullPath.string()) ;


          }
      }


      nChannels = (channels) ? strlen(channels) : 1 ;

      // sort paths

      for( int i=0 ; i<nChannels ; i++ )
          sort(filePaths[i].begin(), filePaths[i].end()) ;

     return (!filePaths[0].empty()) ;
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "image_publisher");
    ros::NodeHandle nh;

    string pubNS = "virtual_cam" ;
    double rate = 20 ;

  //  MonocularImagePublisher pub(paths, "", "", 30) ;

    vector<string> files[2] ;
    int nChannels ;

    parse("/home/malasiot/images/clothes/on_table/kinect_grab_%06d_%c.*", files, nChannels, "cd") ;

    RGBDImagePublisher pub(files[0], files[1], "", "", "rgb_optical_frame",  "depth_optical_frame", true) ;





}
