#include <ros/ros.h>
//#include <ros/single_subscriber_publisher.h>
#include <certh_ps/PhotometricStereo.h>

#include <iostream>
#include <string>
#include <opencv2/opencv.hpp>

#include "utilities.h"
#include <cerrno>

using namespace std;
using namespace cv;

int serialPort;

bool do_reconstruct(certh_ps::PhotometricStereo::Request &req,   certh_ps::PhotometricStereo::Response &res) {

  // Open the camera
  cv::VideoCapture capture(0);

  // check if video successfully opened
  if (!capture.isOpened())
    {
      cout << "Error opening camera." << endl;
      return -1;
    }
  wait(2);
  double rate = 30.0;
  cv::Mat frame; // current video frame
  cv::namedWindow("Extracted Frame");

  //Delay between each frame in ms corresponds to frame rate
  int delay = 1000 / rate;
  string leds[] = {"1", "2", "3", "4", "5", "6", "7", "8"};
  int i = 0;
    
  // throw away the first two frames
  capture.read(frame);
  capture.read(frame);

  // for all frames in video
  for(i=0; i < 8; i++) {
    write(serialPort, "0", 1);
    write(serialPort, leds[i].c_str(), 1);
    // read next frame if any
    if (!capture.read(frame))
      break;
    cv::imshow("Extracted Frame", frame);
    cv::imwrite(leds[i] + ".png", frame);
    // introduce a delay or press key to stop
    if (cv::waitKey(delay) > 0)
      break;
  }

  write(serialPort, "0", 1); // turn off LEDs
  // Close the video file.
  // Not required since called by destructor
  cv::destroyAllWindows();
  capture.release(); frame.release();
  
  res.a = 16;

  return true;
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "photometric_stereo");
  ros::NodeHandle nh;

  // Initialize the Arduino
  serialPort = open("/dev/ttyUSB1", O_RDWR | O_NOCTTY | O_NDELAY);
  
  if (serialPort < 0) {
      cout << strerror(errno) << endl;
    }

  init_port(&serialPort, 19200);
  if(serialPort == 0)
    {
      cout << "Error opening serial port." << endl;
      return -1;
    }
  write(serialPort, "0", 1);
  cout << "Reset lights." << endl;
  
  // Set camera parameters
  system("./cameraSettings.sh");


  //  PhotometricStereoServer server ;
  
  // Register the service with the master
  ros::ServiceServer psServer = nh.advertiseService("PS_reconstruction", &do_reconstruct);
  ROS_INFO("PS service is ready.");
  ros::spin();

  return 0;
}


/*class PhotometricStereoServer {
public:
  PhotometricStereoServer() {

    ros::NodeHandle nh;
    ros::ServiceServer service = nh.advertiseService("reconstruct", &PhotometricStereoServer::do_reconstruct, this);

    pub = nh.advertise<sensor_msgs::PointCloud2>("point_cloud", 1,
						 boost::bind(&PhotometricStereoServer::connectCallback, this, _1),
						 boost::bind(&PhotometricStereoServer::disconnectCallback, this, _1)) ;


  }

private:
 
  bool do_reconstruct(certh_ps::PhotometricStereo::Request &req,   certh_ps::PhotometricStereo::Response &res) {

    return true;

  }

  void connectCallback(const ros::SingleSubscriberPublisher &pub_) {
    if ( pub.getNumSubscribers() > 0 )
      {
	
      }
  }

  void disconnectCallback(const ros::SingleSubscriberPublisher &pub) {

  }

  ros::Publisher pub ;

  };*/
