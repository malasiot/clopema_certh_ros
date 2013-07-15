#include "ros/ros.h"


#include <robot_helpers/Robot.h>
#include <robot_helpers/Utils.h>

#include <Eigen/Geometry>

#include <kinematics_msgs/GetConstraintAwarePositionIK.h>
//#include <planning_environment/models/collision_models.h>
#include <planning_environment/models/model_utils.h>
#include <arm_navigation_msgs/convert_messages.h>

using namespace robot_helpers ;
using namespace std;

bool moveArmHoriz(MoveRobot &cmove, const std::string &armName, double X, double Y, double Z, double theta = 0.0)
{


    Eigen::Quaterniond q ;

    if ( armName == "r2" )
        q = Eigen::AngleAxisd(theta, Eigen::Vector3d::UnitX()) * Eigen::AngleAxisd(-M_PI/2, Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(-M_PI/2, Eigen::Vector3d::UnitZ());
    else
        q = Eigen::AngleAxisd(theta, Eigen::Vector3d::UnitX()) * Eigen::AngleAxisd(M_PI/2, Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(M_PI/2, Eigen::Vector3d::UnitZ());

    return moveGripper(cmove, armName, Eigen::Vector3d(X, Y, Z), q) ;
}

bool ik_tip_link(const string &armName, geometry_msgs::Pose &pose, sensor_msgs::JointState &state) {

    kinematics_msgs::GetConstraintAwarePositionIK ik;
    ik.request.timeout = ros::Duration(5.0);
    ik.request.ik_request.ik_link_name = armName + "_tip_link";
    ik.request.ik_request.pose_stamped.header.frame_id = "base_link";
    ik.request.ik_request.pose_stamped.header.stamp = ros::Time::now();

    getRobotState(ik.request.ik_request.robot_state) ;

    ik.request.ik_request.ik_seed_state = ik.request.ik_request.robot_state;
    ik.request.ik_request.pose_stamped.pose = pose;

    string serviceName = "/clopema_" + armName + "_arm_kinematics/get_constraint_aware_ik" ;
// /clopema_r1_arm_kinematics/get_constraint_aware_ik
    ros::service::waitForService(serviceName) ;

    if (ros::service::call(serviceName, ik)) {
        if (ik.response.error_code.val == ik.response.error_code.SUCCESS)
            ROS_INFO_STREAM("IK Solution found");
        else
            ROS_WARN_STREAM("Error code: " << ik.response.error_code.val<<": "<<arm_navigation_msgs::armNavigationErrorCodeToString(ik.response.error_code));
    } else
        ROS_ERROR("Can't call service");

    state = ik.response.solution.joint_state ;
}

using namespace Eigen ;

geometry_msgs::Pose eigenPoseToROS(const Vector3d &pos, const Quaterniond &orient)
{
    geometry_msgs::Pose pose ;

    pose.position.x = pos.x() ;
    pose.position.y = pos.y() ;
    pose.position.z = pos.z() ;

    pose.orientation.x = orient.x() ;
    pose.orientation.y = orient.y() ;
    pose.orientation.z = orient.z() ;
    pose.orientation.w = orient.w() ;

    return pose ;

}

bool moveWithJointConstraints(const string &armName, const Vector3d &pos, const Quaterniond &orient)
{

    MoveRobot cmove ;

    geometry_msgs::Pose pose_ = eigenPoseToROS(pos, orient);

    sensor_msgs::JointState goal_state ;

    if ( ! ik_tip_link(armName, pose_, goal_state) ) return false ;

    clopema_arm_navigation::ClopemaMotionPlan mp;
    mp.request.motion_plan_req.group_name = armName + "_arm";
    mp.request.motion_plan_req.allowed_planning_time = ros::Duration(5.0);

    if (!getRobotState(mp.request.motion_plan_req.start_state)) return false ;

    vector<string> joint_names ;

    joint_names.push_back(armName + "_joint_s");
    joint_names.push_back(armName + "_joint_l");
    joint_names.push_back(armName + "_joint_u");
    joint_names.push_back(armName + "_joint_r");
    joint_names.push_back(armName + "_joint_b");
    joint_names.push_back(armName + "_joint_t");
    
    mp.request.motion_plan_req.goal_constraints.joint_constraints.resize(joint_names.size());

    for (unsigned int i = 0; i < mp.request.motion_plan_req.goal_constraints.joint_constraints.size(); ++i) {
        mp.request.motion_plan_req.goal_constraints.joint_constraints[i].joint_name = joint_names[i];
        mp.request.motion_plan_req.goal_constraints.joint_constraints[i].position = 0.0;
        mp.request.motion_plan_req.goal_constraints.joint_constraints[i].tolerance_below = 0.1;
        mp.request.motion_plan_req.goal_constraints.joint_constraints[i].tolerance_above = 0.1;
    }

    if (!plan(mp)) return false ;

    control_msgs::FollowJointTrajectoryGoal goal;
    goal.trajectory = mp.response.joint_trajectory;
    cmove.doGoal(goal);

    mp.request.motion_plan_req.group_name = "ext_axis";
    mp.request.motion_plan_req.allowed_planning_time = ros::Duration(5.0);
    if (!getRobotState(mp.request.motion_plan_req.start_state)) return false ;

    joint_names.clear() ;
    joint_names.push_back("ext_axis") ;

    mp.request.motion_plan_req.goal_constraints.joint_constraints.resize(joint_names.size());
    for (unsigned int i = 0; i < mp.request.motion_plan_req.goal_constraints.joint_constraints.size(); ++i) {
        mp.request.motion_plan_req.goal_constraints.joint_constraints[i].joint_name = joint_names[i];
        mp.request.motion_plan_req.goal_constraints.joint_constraints[i].position = 0.0;
        mp.request.motion_plan_req.goal_constraints.joint_constraints[i].tolerance_below = 0.1;
        mp.request.motion_plan_req.goal_constraints.joint_constraints[i].tolerance_above = 0.1;
    }

    if ( !plan(mp) ) return false ;


    goal.trajectory = mp.response.joint_trajectory;
    cmove.doGoal(goal);

    return true ;
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "qt_ros_test") ;
    ros::NodeHandle nh ;

    MoveRobot mv ;

  //  moveWithJointConstraints("r2", Vector3d(-0.5, -0.4, 0.9),  Eigen::AngleAxisd(-M_PI/2, Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(-M_PI/2, Eigen::Vector3d::UnitZ())) ;

    moveArmHoriz(mv, "r2", -0.5, -0.4, 0.9) ;

    addConeToCollisionModel("r1", 0.6, 0.2) ;

    moveArmHoriz(mv, "r2", -0.5, -1.25, 0.9) ;

    resetCollisionModel() ;

    moveArmHoriz(mv, "r2", -0.5, -0.6, 0.9) ;
}


