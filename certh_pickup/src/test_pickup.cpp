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
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>

using namespace std ;
using namespace certh_libs ;
using namespace robot_helpers ;
using namespace std ;
using namespace Eigen ;


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

bool isGraspingSucceeded(cv::Mat depth1, cv::Mat depth2, string armName, float threshold){

    vector <float> diff ;

    for ( int i = 0; i < 400 ; i++){
        for ( int j = 0; j < 400 ; j++){

            diff.push_back ( ((float)depth1.at<unsigned short>(i,j) - (float)depth2.at<unsigned short>(i,j)) * (float)(depth1.at<unsigned short>(i,j) - (float)depth2.at<unsigned short>(i,j))  ) ;

        }
    }
     float sum = 0, meanDepthDiff = 0;

    for ( int i = 0; i < 150000 ; i++)
        sum += diff[i] ;

    meanDepthDiff = (float)sum / 150000.0;
    cout << meanDepthDiff << endl;
    if (meanDepthDiff > threshold)
        return true ;
    openG2() ;
    setGripperState(armName, true);
    return false ;

}

const double defaultDist = 1.0 ;
const double defaultTableHeight = 0.75 ;
//const double defaultTableHeight = 0.075 ;
const double gripperOffset = 0.35 ;


Affine3d getSensorPose(const tf::TransformListener &listener, const string &camera)
{
    tf::StampedTransform transform;
    Affine3d pose ;

    try {
        listener.waitForTransform(camera + "_rgb_optical_frame", "base_link", ros::Time(0), ros::Duration(1) );
        listener.lookupTransform(camera + "_rgb_optical_frame", "base_link", ros::Time(0), transform);

        tf::TransformTFToEigen(transform, pose);

        return pose ;
    } catch (tf::TransformException ex) {
        ROS_ERROR("%s",ex.what());
        return pose ;
    }
}

double getTableHeightFromPlane(const Vector3d &n, double d, const Affine3d &frame)
{
    return n.dot(frame.translation()) + d ;
}

cv::Mat clr_ ;

Vector3d retargetSensor(const cv::Mat dmap, const vector<cv::Point> &hull, const image_geometry::PinholeCameraModel &cm, double Zd)
{
    double minZ, maxZ, Z ;

    cv::minMaxLoc(dmap, &minZ, &maxZ) ;

    // compute the minimum distance of the sensor from the table

    minZ = maxZ/1000.0 + 0.7 ;

    // find the center of the object and re-target the sensor

    cv::Point2f center ;
    float rad ;
    cv::minEnclosingCircle(hull, center, rad) ;

    double zS = Zd * rad / std::min(dmap.cols, dmap.rows) ;

    cv::circle(clr_, center, rad, cv::Scalar(255, 0, 255)) ;

    zS = std::max(minZ, zS) ;

    zS = 0.7 + maxZ/1000 ;

    cv::Point3d p = cm.projectPixelTo3dRay(cv::Point2d(center.x, center.y));

    p.x *= zS ; p.y *= zS ; p.z *= zS ;

    cv::imwrite("/tmp/oo.png", clr_);

    return Vector3d(p.x, p.y, p.z) ;

}

int main(int argc, char **argv) {


    ros::init(argc, argv, "unfolding") ;
    ros::NodeHandle nh ;

    system("/home/akargakos/ROS/clopema_certh_ros/certh_scripts/./openXtion2.sh &");
    sleep(5);

    string armName = "r2", otherArm = "r1" ;
    bool grasp = false ;
    string camera = "xtion2" ;

    //setGripperState(armName, false) ;
    openG2() ;
    setGripperState(armName, true) ;
    MoveRobot cmove ;
    cmove.setServoMode(false) ;
    moveGripperPointingDown(cmove, armName, 1.2, 0, 1.3 ) ;
    vector <geometry_msgs::Pose> poses;
    camera_helpers::OpenNICaptureRGBD grabber("xtion2") ;
    grabber.connect() ;
    ros::Duration(0.3).sleep() ;

    cv::Mat rgb, depth, tmpDepth;

    ros::Time ts ;
    image_geometry::PinholeCameraModel cm ;

    if(!grabber.grab(rgb, depth, ts, cm)){
       cout<<" Cant grab image!!  " << endl ;
       return false ;
    }

    tmpDepth = depth ;

    ObjectOnPlaneDetector objDet(depth, cm.fx(), cm.fy(), cm.cx(), cm.cy()) ;

    Eigen::Vector3d n ;
    double d ;

    if ( !objDet.findPlane(n, d) ) return 0 ;

    vector<cv::Point> hull ;
    cv::Mat dmap ;
    cv::Mat mask = objDet.findObjectMask(n, d, 0.01, dmap, hull) ;

    // Sensor planning

    tf::TransformListener listener ;
    Affine3d frame = getSensorPose(listener, camera) ;

    Vector3d t = retargetSensor(dmap, hull, cm, defaultDist - defaultTableHeight) ;

    Vector3d target = frame.inverse() * t ;

    double newDist = target.z() +  0.7 ;

    //double newDist = t.z() + defaultTableHeight - gripperOffset ;



    trajectory_msgs::JointTrajectory traj ;
    if ( robot_helpers::planXtionToPose(armName, Vector3d(target.x(), target.y(), newDist ), robot_helpers::lookAt(Vector3d(0, 0, -1)), traj) )
    {
        cmove.execTrajectory(traj) ;
    }

    ///////////////////////////////////////////////////////////////////////

    while (!grasp){

        if(!grabber.grab(rgb, depth, ts, cm)){
           cout<<" Cant grab image!!  " << endl ;
           return false ;
        }

        ObjectOnPlaneDetector objDet2(depth, cm.fx(), cm.fy(), cm.cx(), cm.cy()) ;


        if ( !objDet2.findPlane(n, d) ) return 0 ;

        hull.clear() ;
        cv::Mat dmap2 ;
        mask = objDet2.findObjectMask(n, d, 0.01, dmap2, hull) ;

        RidgeDetector rdg ;
        vector<RidgeDetector::GraspCandidate> gsp ;

        rdg.detect(dmap2, gsp) ;
        rdg.draw(rgb, gsp) ;

        cv::imwrite("/tmp/gsp.png", rgb) ;

        for(unsigned int i = 0 ; i < gsp.size() ; i++ ){

            poses.clear();
            unsigned short zval = depth.at<ushort>(gsp[i].y, gsp[i].x) ;

            cv::Point3d val = cm.projectPixelTo3dRay(cv::Point2d(gsp[i].x, gsp[i].y));

            val.x *= zval/1000.0 ;
            val.y *= zval/1000.0 ;
            val.z *= zval/1000.0 ;

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

            poses.push_back(pose);
            pose.position.z -= offset +0.01 ;
            poses.push_back(pose);
            cout<< " GRASPING POINT = "<< pose.position.x << " " << pose.position.y << " "  <<pose.position.z << endl ;

            if ( moveArmThrough(poses, armName) == -1 ){
                cout<< "cant make the grasp"<< endl ;
                continue ;
            }
            cout << "dominant grasping point i = " <<  i << endl;

            break;
        }

        setGripperState( armName, false) ;

        moveGripperPointingDown(cmove, armName, 1.2, 0, 1.3 ) ;

        if(!grabber.grab(rgb, depth,  ts, cm)){
            cout<<" Cant grab image!!  " << endl ;
            return false ;
        }

        if ( isGraspingSucceeded(tmpDepth, depth, armName, 60000) )
            grasp = true ;
    }

    moveHomeArm( armName) ;
    system("/home/akargakos/ROS/clopema_certh_ros/certh_scripts/./killXtion2.sh") ;

    return 0 ;

}
