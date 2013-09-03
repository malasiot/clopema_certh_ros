#include "ObjectOnPlaneDetector.h"
#include "RidgeDetector.h"
#include <highgui.h>
#include <iostream>
#include <robot_helpers/Robot.h>
#include <robot_helpers/Utils.h>
#include <robot_helpers/Geometry.h>
#include <certh_libs/Point2D.h>
#include <camera_helpers/OpenNICapture.h>
#include <clopema_motoros/WriteIO.h>

using namespace std ;
using namespace certh_libs ;
using namespace robot_helpers ;
using namespace std;

Eigen::Vector3d findTarget(float x , float y, pcl::PointCloud<pcl::PointXYZ> pc ){

    Eigen::Vector3d v;



}


int openG2(){

    clopema_motoros::WriteIO openGripper;
    openGripper.request.address = 10026;

    for(unsigned int i = 0 ; i < 3 ; i++){
        openGripper.request.value = false;
        if (!ros::service::call("/write_io", openGripper)) {
            ROS_ERROR("Can't call service write_io");
            return -1;
        }
        ros::Duration(0.1).sleep();

        openGripper.request.value = true;
        ros::service::waitForService("/write_io");
        if (!ros::service::call("/write_io", openGripper)) {
            ROS_ERROR("Can't call service write_io");
            return -1;
        }
        ros::Duration(0.5).sleep();
    }
    openGripper.request.value = false;
    if (!ros::service::call("/write_io", openGripper)) {
        ROS_ERROR("Can't call service write_io");
        return -1;
    }
    ros::Duration(0.1).sleep();

}
int main(int argc, char **argv) {


    ros::init(argc, argv, "unfolding");
    ros::NodeHandle nh;


//    cv::Mat clr = cv::imread("/home/malasiot/images/clothes/calibration/on_table/cap_3/cap_rgb_000009.png") ;
//    cv::Mat depth = cv::imread("/home/malasiot/images/clothes/calibration/on_table/cap_3/cap_depth_000009.png", -1) ;
//
    string armName = "r2";
    setGripperState(armName, false);


    MoveRobot cmove;
    cmove.setServoMode(false);
    moveGripperPointingDown(cmove, armName, 0, -1.1, 1.3 );

    camera_helpers::OpenNICaptureAll grabber("xtion2");
    grabber.connect();
    ros::Duration(0.3).sleep();

    cv::Mat rgb, depth;
    pcl::PointCloud<pcl::PointXYZ> pc;
    ros::Time ts ;
    image_geometry::PinholeCameraModel cm;

    if(!grabber.grab(rgb, depth, pc, ts, cm)){
        cout<<" Cant grab image!!  " << endl;
        return false;
    }

  //  grabber.disconnect();
//
    ObjectOnPlaneDetector det(depth, 525, 525, 640/2-0.5, 480/2-0.5) ;

    Eigen::Vector3d n ;
    double d ;

    cv::Mat inliers ;

    det.findPlane(n, d, inliers) ;

    vector<cv::Point> hull, hull2 ;
    cv::Mat dmap ;
    cv::Mat mask = det.findObjectMask(n, d, 0.01, dmap, hull) ;
    cv::Mat ref = det.refineSegmentation(rgb, mask, hull2) ;

    RidgeDetector rdg ;
    vector<RidgeDetector::GraspCandidate> gsp ;

    rdg.detect(dmap, gsp) ;
    rdg.draw(rgb, gsp) ;


    cv::imwrite("/tmp/gsp.png", rgb) ;
    //cout << "point is " <<gsp[0].x << " "<<  gsp[0].y << endl;

    ////////////////
    pcl::PointXYZ val = pc.at(gsp[0].x, gsp[0].y) ;

    Eigen::Vector3d p(val.x, val.y, val.z) ;
    cout << p << endl;
    Eigen::Matrix4d calib = getTranformationMatrix("xtion2_rgb_optical_frame");
    Eigen::Vector4d tar(p.x(), p.y(), p.z(), 1);
    Eigen::Vector4d targetP;

    targetP = calib * tar;
    //cout << targetP << endl;
    cout << (gsp[0].alpha) <<  " = angle " << endl;
    rotateGripper(M_PI-gsp[0].alpha,armName);

    geometry_msgs::Pose pose;
    float offset= 0.05;
    pose.position.x = targetP.x();
    pose.position.y = targetP.y();
    pose.position.z = targetP.z()+offset;
    pose.orientation = getArmPose(armName).orientation;
    if ( moveArm(pose, armName) == -1)
        cout<< "cant make 1st move"<< endl;


    pose = getArmPose(armName, armName + "_ee");
    pose.position.x += 0.02;
    if ( moveArm(pose, armName, armName + "_ee") == -1 )
        cout<< "cant make 2nd move"<< endl;

    setGripperState(armName, true);
    openG2();
    setGripperState(armName, true);

    pose = getArmPose(armName);
    pose.position.z -= offset;
    cout<< " target = "<< pose.position.x << " " << pose.position.y << " "  <<pose.position.z << endl;

    if ( moveArm(pose, armName) == -1 )
        cout<< "cant make 3rd move"<< endl;

    setGripperState(armName, false);
    setGripperState(armName, false);
    setGripperState(armName, false);

    moveHomeArm(armName);

    setServoPowerOff();
    /////////////////
    return 0;

}
