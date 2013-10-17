#include <ros/ros.h>
//#include <ros/single_subscriber_publisher.h>
#include <certh_ps/PhotometricStereo.h>

#include <iostream>
#include <string>
#include <sstream>
#include <opencv2/opencv.hpp>

#include "utilities.h"
#include <cerrno>

#include <boost/lexical_cast.hpp>
#include <fstream>

using namespace cv;
using namespace std;

int serialPort;
cv::VideoCapture *capture;
ros::Publisher pub;

bool do_reconstruct(certh_ps::PhotometricStereo::Request &req,   certh_ps::PhotometricStereo::Response &res) {

  // Paths and Filenames of various files
  string lightsPath = "data/Lights/L.dat"; //Lights direction path
  string fullImagePath = "data/Full_Images/"; //Place to save full images
  string multiImagePath = "data/Images/"; // Place to save images from single leds
  string normalsPath = "data/Normals/"; // Place to save normals
  string albedoPath = "data/Albedo/"; // Place to save albedo
  string depthPath = "data/Points/"; // Place to save depth
  string FF_Imagepath = "data/FF/I_FF_";
  string imageFilenames;

  // Quantities required for PS
  Mat Lights; // Lights directions
  int imageNum = 8; // Number of Images
  int x0 = 120, y0 = 40, w = 400; //Region of interest / bounding box
  Rect roi(x0, y0, w, w); // Region of interest
  Mat imageAcquired[imageNum]; // Original image to start process
  Mat FF_Image; // Flatfielding image
  Mat FFed_Image[imageNum]; // Flatfielded image
  int count; // Counter
  Mat Nx, Ny, Nz, alb_gr; // Normals and Albedo
  Mat Z; // Depth values

  // Load the Lights directions file. Lights = [imageNumx3 matrix]
  Lights = loadLights(imageNum, lightsPath);
  cout << Lights << endl;
  int i, j;  
  /*double L[8][3];
  Lights = Mat_<double>(8, 3);

  ifstream f("Lights.txt");
  cout << "Lights matrix:" << endl;
  for (i = 0; i < 8 ; i++) {
    for (j = 0; j < 3; j++) {
      f >> L[i][j];
      Lights.row(i).col(j) = L[i][j];
      cout << L[i][j] << '\t';
    }
    cout << '\n';
  }
  */
  //  cout << Lights;

  double rate = 30.0;
  cv::Mat frame; // current video frame

  //Delay between each frame in ms corresponds to frame rate
  int delay = 1000 / rate;

  system("v4l2-ctl -c focus_auto=1"); // turn auto-focus on
  //  system("v4l2-ctl -c exposure_auto=3"); // turn auto-exposure on

  write(serialPort, "0", 1); 
  write(serialPort, "1", 1); // turn on the first LED for auto-focus
  
  stringstream ss;
  cv::Mat images[8];
  cv::Mat ff_images[8];

  // wait for 3 seconds to focus 
  for(i = 0; i < 90; i++) {
    // read next frame if any
    if (!capture->read(frame))
      break;
    //    cv::imshow("Preview", frame);
    // introduce a delay or press key to stop
    if (cv::waitKey(delay) > 0)
      break;
  }
  system("v4l2-ctl -c focus_auto=0"); // turn auto-focus off
  //  system("v4l2-ctl -c exposure_auto=1"); // turn auto-exposure off
  
  // for all frames in video
  for(i=0; i < 8; i++) {
    write(serialPort, "0", 1);
    ss << i + 1 << '\n';
    write(serialPort, ss.str().c_str(), 1);
    //    ss.str(std::string());
    //    ss.clear()f;
    for(j = 0; j < 6; j++) {
      // read next frame if any
      if (!capture->read(frame))
	break;
      //    cv::imshow("Extracted Frame", frame);
      //    cv::imwrite(leds[i].substr(0, leds[i].size()-1) + ".jpg", frame);
      //ss << i*8 + j;

    // introduce a delay or press key to stop
      if (cv::waitKey(delay) > 0)
	break;
    }
  
    cvtColor(frame(roi), images[i], CV_RGB2GRAY);
    //    images[i] = frame;

    cv::imwrite("data/images/" + ss.str().substr(0, ss.str().size() - 1) + ".jpg", images[i]);
    ss.str(std::string());
    ss.clear();
  }

  write(serialPort, "0", 1);  

  // FlatFielding
  //string data_path = "/home/christos/clopema/certh_ps/data/";
  string ff_path =  "data/ff/";
  for(i = 0; i < 8; i++) {
    string path = ff_path +  boost::lexical_cast<string>(i+1) + ".jpg";
    FF_Image = imread(path, 0);
    FFed_Image[i] = flatFielding(w, FF_Image, images[i], "normalize"); // fitting or normalize
  }
  

  // Photometric stereo algorithm
  photometricStereo(Lights, FFed_Image, w, 0, 255, imageNum, Nx, Ny, Nz, alb_gr);
  //  photometricStereo(Lights, images, w, 0, 255, imageNum, Nx, Ny, Nz, alb_gr); 

  cv::imwrite("data/normals/Nx.jpg", Nx*255.0);
  cv::imwrite("data/normals/Ny.jpg", Ny*255.0);
  cv::imwrite("data/normals/Nz.jpg", Nz*255.0);
  cv::imwrite("data/albedo/albedo.jpg", alb_gr*255.0);

  cv::FileStorage fs("data.xml", cv::FileStorage::WRITE);
  fs << "Nx" <<  Nx << "Ny" << Ny << "Nz" << Nz << "albedo" << alb_gr;
  
  // Photometric Stereo - XYZ points
  Z = psCloud(w, Nx, Ny, Nz);

  fs << "Z" << Z;
  fs.release();
  
  // Cloud Visualization
  cloudVis(alb_gr, Z, w, "New");
 
  res.a = "Returned successfully.";

  return true;
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "photometric_stereo");
  ros::NodeHandle nh;

  // Initialize the Arduino
  serialPort = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY | O_NDELAY);
  if (serialPort < 0) {
      cout << strerror(errno) << endl;
    }

  init_port(&serialPort, 19200);
  if(serialPort == 0)
    {
      cout << "Error opening serial port." << endl;
      return -1;
    }
  cout << "Serial port initialiazed correctly." << endl;
  write(serialPort, "0", 1);
  
  // Set camera parameters
  cout << "Camera settings :" << endl;
  system("./cameraSettings.sh");

  // ros::Duration(5).sleep();
  
  // Open the camera
  capture = new cv::VideoCapture(0);

  // check if video successfully opened
  if (!capture->isOpened())
    {
      cout << "Error opening camera." << endl;
      return -1;
    }
  

  //PhotometricStereoServer server;
  
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
