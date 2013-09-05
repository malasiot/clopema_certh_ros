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
using namespace std ;

Eigen::Vector3d findTarget(float x , float y, pcl::PointCloud<pcl::PointXYZ> pc ){

    Eigen::Vector3d v ;



}
Eigen::Vector3d vect;


int openG2(){

    clopema_motoros::WriteIO openGripper ;
    openGripper.request.address = 10026 ;

    for(unsigned int i = 0 ; i < 3 ; i++){
        openGripper.request.value = false ;
        if (!ros::service::call("/write_io", openGripper)) {
            ROS_ERROR("Can't call service write_io") ;
            return -1 ;
        }
        ros::Duration(0.1).sleep() ;

        openGripper.request.value = true ;
        ros::service::waitForService("/write_io") ;
        if (!ros::service::call("/write_io", openGripper)) {
            ROS_ERROR("Can't call service write_io") ;
            return -1 ;
        }
        ros::Duration(0.5).sleep() ;
    }
    openGripper.request.value = false ;
    if (!ros::service::call("/write_io", openGripper)) {
        ROS_ERROR("Can't call service write_io") ;
        return -1 ;
    }
    ros::Duration(0.1).sleep() ;

}

geometry_msgs::Quaternion findAngle(float theta, Eigen::Matrix4d calib){

    Eigen::Vector4d norm ;
    norm << cos(theta), sin(theta), 0, 0 ;

    Eigen::Vector4d targetN ;
    targetN = calib * norm.normalized() ;

    Eigen::Matrix3d rotMat;
    Eigen::Vector3d Y(targetN.x(), targetN.y(), targetN.z());
    Eigen::Vector3d X;
    Eigen::Vector3d Z(0, 0, -1);

    X = Y.cross(Z);

    rotMat(0, 0)=X.x();
    rotMat(1, 0)=X.y();
    rotMat(2, 0)=X.z();

    rotMat(0, 1)=Y.x();
    rotMat(1, 1)=Y.y();
    rotMat(2, 1)=Y.z();

    rotMat(0, 2)=Z.x();
    rotMat(1, 2)=Z.y();
    rotMat(2, 2)=Z.z();

    vect << X.x() , X.y() , X.z() ;

    return rotationMatrix3ToQuaternion(rotMat);


}


int main(int argc, char **argv) {


    ros::init(argc, argv, "unfolding") ;
    ros::NodeHandle nh ;

    string armName = "r2", otherArm = "r1" ;

    //system( "roslaunch certh_launch xtion2.launch") ;
    //ros::Duration(5).sleep();

    //setGripperState(armName, false) ;
    openG2() ;
    setGripperState(armName, true) ;
    MoveRobot cmove ;
    cmove.setServoMode(false) ;
    moveGripperPointingDown(cmove, armName, 1.2, 0, 1.3 ) ;

    camera_helpers::OpenNICaptureAll grabber("xtion2") ;
    grabber.connect() ;
    ros::Duration(0.3).sleep() ;

    cv::Mat rgb, depth ;
    pcl::PointCloud<pcl::PointXYZ> pc ;
    ros::Time ts ;
    image_geometry::PinholeCameraModel cm ;

    if(!grabber.grab(rgb, depth, pc, ts, cm)){
        cout<<" Cant grab image!!  " << endl ;
        return false ;
    }

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

    for(unsigned int i = 0 ; i < gsp.size() ; i++ ){

        pcl::PointXYZ val = pc.at(gsp[0].x, gsp[0].y) ;

        Eigen::Vector3d p(val.x, val.y, val.z) ;

        Eigen::Matrix4d calib = getTranformationMatrix("xtion2_rgb_optical_frame") ;
        Eigen::Vector4d tar (p.x(), p.y(), p.z(), 1) ;
        Eigen::Vector4d targetP ;

        targetP = calib * tar ;

        float offset= 0.10 ;

        geometry_msgs::Pose pose ;

        pose.orientation = findAngle(gsp[0].alpha, calib) ;
        pose.position.x = targetP.x(); //+ vect.x() * 0.01 ;
        pose.position.y = targetP.y(); //+ vect.y() * 0.01;
        pose.position.z = targetP.z()+ offset; //+ vect.z() * 0.01 ;
        cout<< "vector = " << vect.x() << " "<< vect.y() <<" "<< vect.z() << endl;

        if ( moveArm(pose, armName) == -1){
            cout<< "cant make 1st move"<< endl ;
            continue ;
        }

//        pose = getArmPose(armName, armName + "_ee") ;
//        pose.position.x += 0.01 ;

//        if ( moveArm(pose, armName, armName + "_ee") == -1 ){
//            cout<< "cant make 2nd move"<< endl ;
//            continue ;
//        }

        pose = getArmPose(armName) ;
        pose.position.z -= offset +0.01 ;
        cout<< " GRASPING POINT = "<< pose.position.x << " " << pose.position.y << " "  <<pose.position.z << endl ;

        if ( moveArm(pose, armName) == -1 )
            cout<< "cant make 3rd move"<< endl ;

        cout << "dominant grasping point i = " <<  i << endl;

        break;
    }

    setGripperState( armName, false) ;
    moveHomeArm( armName) ;
    setServoPowerOff() ;

    return 0 ;

}
