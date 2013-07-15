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


namespace robot_helpers {


class Unfold {

public:
    Unfold();
    virtual ~Unfold();

private:

    std::string holdingArm;
    std::string movingArm;

    void switchArms();
    void setHoldingArm(const std::string &armName);
    int moveArmConstrains(geometry_msgs::Pose pose, const std::string &armName, float radious,  geometry_msgs::Quaternion q);
    void setPathConstraints(clopema_arm_navigation::ClopemaMotionPlan & mp, float radious , const std::string &armName,  geometry_msgs::Quaternion q);
    void rotateGripper(float angle, const std::string &armName);


    int moveArm(geometry_msgs::Pose pose, const std::string &armName);
    int moveArms( geometry_msgs::Pose pose1, geometry_msgs::Pose pose2);

    int setGripperStates(const std::string &armName  , bool open);
    int setGrippersStates( bool open);

    float getArmsDistance();

    geometry_msgs::Pose getArmPose( const std::string &armName, const std::string &frameName = "base_link");

    void findGraspingOrientation(Eigen::Vector4d vector, Eigen::Matrix4d rotMat);
    float findBias(Eigen::Vector4d vector);
    geometry_msgs::Quaternion rotationMatrixToQuaternion(btMatrix3x3 matrix);

    Eigen::Matrix4d getTranformationMatrix(const std::string &frameName, const std::string &coordSys = "base_link" );
    tf::StampedTransform getTranformation(const std::string &frameName, const std::string &coordSys = "base_link" );


public:

    int GraspLowestPoint(const std::string &armName);

};

}

#endif
