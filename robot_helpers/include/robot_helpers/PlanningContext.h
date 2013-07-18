#ifndef __PLANNING_CONTEXT_H__
#define __PLANNING_CONTEXT_H__

#include <kinematics_base/kinematics_base.h>
#include <arm_navigation_msgs/GetPlanningScene.h>
#include <planning_environment/models/model_utils.h>
#include <visualization_msgs/MarkerArray.h>

#include <robot_helpers/Robot.h>
#include <robot_helpers/Utils.h>
#include <robot_helpers/Geometry.h>

namespace robot_helpers {


class PlanningContext {
public:

    PlanningContext() ;

    bool init(const std::string &groupName) ;

    void setStateFromJointState(const sensor_msgs::JointState &js) ;

    bool isStateValid() ;

    void getRobotMarkers(visualization_msgs::MarkerArray &markers) ;

    void initFromRobotState() ;

    Eigen::Affine3d getTransformToBase(const std::string &linkName) ;

    bool solveIK(const std::string &tipName, const Eigen::Vector3d &pos, const Eigen::Quaterniond &orient, sensor_msgs::JointState &state) ;

    ~PlanningContext() ;

    boost::shared_ptr<planning_environment::CollisionModels> cm_ ;
    planning_models::KinematicState *state_ ;
    std::vector<std::string> joint_names, link_names ;
    boost::shared_ptr<kinematics::KinematicsBase> solver_ ;
};





}

#endif
