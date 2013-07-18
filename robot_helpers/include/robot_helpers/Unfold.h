#ifndef __ROBOT_HELPERS_UNFOLDING_H__
#define __ROBOT_HELPERS_UNFOLDING_H__

#include "robot_helpers/Utils.h"
#include <geometric_shapes/shape_operations.h>
#include <planning_environment/util/construct_object.h>
#include <geometry_msgs/Pose.h>
#include <arm_navigation_msgs/CollisionObject.h>
#include <arm_navigation_msgs/GetPlanningScene.h>
#include <clopema_motoros/SetGripperState.h>
#include <visualization_msgs/MarkerArray.h>
#include <planning_environment/models/collision_models.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>

#include <cv_bridge/cv_bridge.h>

#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/ros/conversions.h>


#include <nodelet/nodelet.h>
#include <nodelet/loader.h>
#include <iostream>
#include <fstream>
#include <ros/ros.h>
#include <camera_helpers/OpenNICapture.h>


#include <cv.h>
#include <Eigen/Core>
#include <Eigen/Eigenvalues> // for cwise access

using namespace std;

namespace robot_helpers {


class Unfold {

public:
    Unfold(const string &armName);
    virtual ~Unfold();

private:

    string holdingArm;
    string movingArm;
public:
    inline Eigen::Matrix3d vertical() {
        Eigen::Matrix3d vertical;
        if(holdingArm == "r1"){
            vertical << 0, -1, 0,-1, 0, 0, 0, 0, -1;
        }
        else{
            vertical << 0, 1, 0, 1, 0, 0, 0, 0, -1;
        }
        return vertical;
    }


    inline geometry_msgs::Pose holdingArmPose(){
        geometry_msgs::Pose pose;
        pose.orientation = rotationMatrix3ToQuaternion(vertical());
        if (holdingArm == "r1"){
            pose.position.x = -0.12;
            pose.position.y = -0.83;
            pose.position.z = 1.6;
        }
        else{
            pose.position.x = -0.23;
            pose.position.y = -0.8;
            pose.position.z = 1.6;
        }
        return pose;
    }

    inline geometry_msgs::Pose movingArmPose(){
        geometry_msgs::Pose pose;
        pose.orientation = rotationMatrix3ToQuaternion(vertical());
        if (holdingArm == "r1"){
            pose.position.x = 0.7 ;
            pose.position.y = -1;
            pose.position.z = 1.3;
        }
        else{
            pose.position.x = -0.7 ;
            pose.position.y = -0.5;
            pose.position.z = 1.5;
        }
        return pose;
    }


    void switchArms();
    void setHoldingArm(const string &armName);
    int moveArmConstrains(geometry_msgs::Pose pose, const string &armName, float radious);
    void setPathConstraints(clopema_arm_navigation::ClopemaMotionPlan & mp, float radious , const string &armName,  geometry_msgs::Quaternion q);
    void rotateGripper(float angle, const string &armName);
    int moveArmBetweenSpheres( string armName, bool up, geometry_msgs::Pose goalPose);
    int moveArm(geometry_msgs::Pose pose, const string &armName);
    int moveArms( geometry_msgs::Pose pose1, geometry_msgs::Pose pose2,  const string &arm1Name = "r1", const string &arm2Name = "r2");
    int moveArmsNoTearing( geometry_msgs::Pose pose1, geometry_msgs::Pose pose2,  const string &arm1Name = "r1", const string &arm2Name = "r2");
    bool moveHomeArm(const string &armName);
    int moveArmThrough(vector <geometry_msgs::Pose> poses , const string &armName);
    void grabFromXtion(cv::Mat rgb, cv::Mat depth, pcl::PointCloud<pcl::PointXYZ> pc);

    int setGripperStates(const string &armName  , bool open);
    int setGrippersStates( bool open);

    float getArmsDistance();

    geometry_msgs::Pose getArmPose( const string &armName, const string &frameName = "base_link");

    Eigen::Matrix4d findGraspingOrientation(Eigen::Vector4d vector );
    float findBias(Eigen::Vector4d vector);
    geometry_msgs::Quaternion rotationMatrix4ToQuaternion(Eigen::Matrix4d matrix);
    geometry_msgs::Quaternion rotationMatrix3ToQuaternion(Eigen::Matrix3d matrix);
    bool findLowestPoint(const pcl::PointCloud<pcl::PointXYZ> &depth, const Eigen::Vector3f &orig, const Eigen::Vector3f &base, float apperture, Eigen::Vector3f &p, Eigen::Vector3f &n, cv::Mat depthMap);
    Eigen::Vector3f computeNormal(const pcl::PointCloud<pcl::PointXYZ> &pc, int x, int y);
    void robustPlane3DFit(vector<Eigen::Vector3f> &x, Eigen::Vector3f  &c, Eigen::Vector3f &u);


    Eigen::Matrix4d getTranformationMatrix(const string &frameName, const string &coordSys = "base_link" );
    tf::StampedTransform getTranformation(const string &frameName, const string &coordSys = "base_link" );


public:

    int GraspLowestPoint(bool lastMove = false);
    void rotateHoldingGripper(float angle);

};

void printPose(geometry_msgs::Pose p);

}

#endif
