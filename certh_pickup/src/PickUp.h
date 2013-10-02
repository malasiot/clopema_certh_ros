#ifndef PICKUP_H__
#define PICKUP_H__

#include <robot_helpers/Utils.h>
#include <robot_helpers/Geometry.h>
#include <camera_helpers/OpenNICapture.h>
#include <clopema_motoros/SetGripperState.h>
#include <certh_libs/ObjectOnPlaneDetector.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <highgui.h>
#include <certh_libs/RidgeDetector.h>

using namespace std ;
using namespace robot_helpers ;
using namespace Eigen ;

class PickUp {

public:

    PickUp(string arm) ;
    virtual ~PickUp() ;
    string camera;

    bool moveAboveTable( ) ;
    bool moveXtionAboveCloth() ;
    bool graspClothFromTable(const ros::Duration &dur = ros::Duration(100)) ;



private:


    Eigen::Vector3d parkingPose ;

    string armName ;

    cv::Mat prevDepth ;

    double defaultDist ;
    double defaultTableHeight ;

    Vector3d retargetSensor(const cv::Mat dmap, const vector<cv::Point> &hull, const image_geometry::PinholeCameraModel &cm, double Zd) ;
    Affine3d getSensorPose(const tf::TransformListener &listener, const string &camera) ;
    geometry_msgs::Quaternion findAngle(float theta, Eigen::Matrix4d calib);

    bool findClothHull( cv::Mat  &depth, cv::Mat &dmap, cv::Mat &mask, vector<cv::Point> &hull, image_geometry::PinholeCameraModel &cm  ) ;
    bool findGraspCandidates( vector<certh_libs::RidgeDetector::GraspCandidate> &gsp, cv::Mat  &depth, image_geometry::PinholeCameraModel &cm ) ;
    bool graspTargetCandidate( vector<certh_libs::RidgeDetector::GraspCandidate> &gsp, cv::Mat &depth, image_geometry::PinholeCameraModel &cm ) ;
    bool isGraspingSucceeded(float threshold) ;

    int openG2() ;


};

#endif
