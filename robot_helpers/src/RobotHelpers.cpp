#include "robot_helpers/Robot.h"

#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>

#include <clopema_planning_actions/GetRobotState.h>

//#include <clopema_motoros/SetSpeed.h>

using namespace std ;

namespace robot_helpers {

MoveRobot::MoveRobot() :
		cmc_(getServerName().data(), true) {

	if (!cmc_.waitForServer(ros::Duration(60))) {
		ROS_ERROR("Can't connect to server: %s", getServerName().data());
		ros::shutdown();
	}

}

MoveRobot::~MoveRobot() {
}

void MoveRobot::doGoal(const control_msgs::FollowJointTrajectoryGoal & goal) {
    cmc_.sendGoal(goal, boost::bind(&MoveRobot::doneCb, this, _1, _2),
                        boost::bind(&MoveRobot::activeCb, this));
	if (!cmc_.waitForResult(ros::Duration(30)))
		cmc_.cancelGoal();

}

void MoveRobot::activeCb() {
    actionStarted() ;
}


void MoveRobot::doneCb(const actionlib::SimpleClientGoalState& state, const control_msgs::FollowJointTrajectoryResultConstPtr& result) {
	ROS_INFO("Finished in state [%s:%s]", state.toString().c_str(), state.text_.data());
	ROS_INFO("Result - error code: %d", result->error_code);
	setServoPowerOff(false);

    actionCompleted() ;
}

bool MoveRobot::setServoPowerOff(bool force) {
	clopema_motoros::SetPowerOff soff;
	soff.request.force = force;
	ros::service::waitForService("/joint_trajectory_action/set_power_off");
	if (!ros::service::call("/joint_trajectory_action/set_power_off", soff)) {
		ROS_ERROR("Can't call service set_power_off");
		return false;
	}
	return true;
}

bool getRobotState(arm_navigation_msgs::RobotState & rs) {
	ros::service::waitForService("/environment_server/get_robot_state");
	arm_navigation_msgs::GetRobotState r;
	if (!ros::service::call("/environment_server/get_robot_state", r)) {
		ROS_ERROR("Can't get current robot state.");
		return false;
	}
	rs = r.response.robot_state;
	return true;
}

bool plan(clopema_arm_navigation::ClopemaMotionPlan& mp) {
	ros::service::waitForService("/clopema_planner/plan");
	if (!ros::service::call("/clopema_planner/plan", mp)) {
		ROS_ERROR("Can't call service clopema_planner/plan");
		return false;
	}

	if (mp.response.error_code.val == arm_navigation_msgs::ArmNavigationErrorCodes::SUCCESS) {
		return true;
	} else {
		ROS_WARN("Can't plan trajectory: %d - %s", mp.response.error_code.val, arm_navigation_msgs::armNavigationErrorCodeToString(mp.response.error_code).data());
		return false;
	}
}

std::vector<std::string> getJointsInGroup(std::string group) {
	planning_environment::CollisionModels cm("robot_description");
	return cm.getKinematicModel()->getModelGroup(group)->getJointModelNames();
}

void poseToClopemaMotionPlan(clopema_arm_navigation::ClopemaMotionPlan& mp, arm_navigation_msgs::SimplePoseConstraint & desired_pose) {
	arm_navigation_msgs::PositionConstraint position_constraint;
	arm_navigation_msgs::OrientationConstraint orientation_constraint;
	arm_navigation_msgs::poseConstraintToPositionOrientationConstraints(desired_pose, position_constraint, orientation_constraint);
	mp.request.motion_plan_req.goal_constraints.orientation_constraints.push_back(orientation_constraint);
	mp.request.motion_plan_req.goal_constraints.position_constraints.push_back(position_constraint);

}


Eigen::Affine3d getPose(const std::string &armName, const std::string &base_link, const ros::Time &ts) {

    string base_link_ ;
    if ( base_link.empty() ) base_link_ = "base_link" ;
    else base_link_ = base_link ;

    tf::TransformListener listener(ros::Duration(1.0));
    tf::StampedTransform transform;
    Eigen::Affine3d pose ;

    try {
        listener.waitForTransform(armName + "_ee", base_link_, ts, ros::Duration(1) );
        listener.lookupTransform(armName + "_ee", base_link_, ts, transform);


        tf::TransformTFToEigen(transform, pose);

        return pose ;
    } catch (tf::TransformException ex) {
        ROS_ERROR("%s",ex.what());
        return pose ;
    }

}

Eigen::Affine3d getCurrentPose(const std::string &armName) {

    Eigen::Affine3d pose ;

    ros::service::waitForService("/clopema_planning_actions/get_robot_state");

    clopema_planning_actions::GetRobotState rs;

    if (!ros::service::call("/clopema_planning_actions/get_robot_state", rs)) {
        ROS_ERROR("Can't get current robot state.");
        return pose ;
    }

    tf::Stamped<tf::Pose> pose_ ;
    if ( armName == "r1" )
        tf::poseStampedMsgToTF(rs.response.robot_1_pose, pose_) ;
    else
        tf::poseStampedMsgToTF(rs.response.robot_2_pose, pose_) ;

    tf::TransformTFToEigen(pose_, pose) ;

    return pose ;

}
/*
bool setRobotSpeed(float speed)
{
    ros::service::waitForService("/set_robot_speed");

    clopema_motoros::SetSpeed::Request req ;
    clopema_motoros::SetSpeed::Response res ;

    req.speed = speed ;

    if (!ros::service::call("/set_robot_speed", req, res)) {
        ROS_ERROR("Can't set robot speed.");
        return false ;
    }
    else return true ;

}
*/

} // namespace robot_helpers

