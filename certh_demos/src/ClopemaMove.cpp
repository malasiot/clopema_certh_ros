/*
 * ClopemaMove.cpp
 *
 *  Created on: Aug 10, 2012
 *      Author: Vladimír Petrík
 */

#include "ClopemaMove.h"

ClopemaMove::ClopemaMove() :
		cmc_(getServerName().data(), true) {
	if (!cmc_.waitForServer(ros::Duration(60))) {
		ROS_ERROR("Can't connect to server: %s", getServerName().data());
		ros::shutdown();
	}

}

ClopemaMove::~ClopemaMove() {
}

void ClopemaMove::doGoal(const control_msgs::FollowJointTrajectoryGoal & goal) {
	cmc_.sendGoal(goal, boost::bind(&ClopemaMove::doneCb, this, _1, _2));
	if (!cmc_.waitForResult(ros::Duration(30)))
		cmc_.cancelGoal();
}

void ClopemaMove::doneCb(const actionlib::SimpleClientGoalState& state, const control_msgs::FollowJointTrajectoryResultConstPtr& result) {
	ROS_INFO("Finished in state [%s:%s]", state.toString().c_str(), state.text_.data());
	ROS_INFO("Result - error code: %d", result->error_code);
	setServoPowerOff(false);
}

bool ClopemaMove::setServoPowerOff(bool force) {
	clopema_motoros::SetPowerOff soff;
	soff.request.force = force;
	ros::service::waitForService("/joint_trajectory_action/set_power_off");
	if (!ros::service::call("/joint_trajectory_action/set_power_off", soff)) {
		ROS_ERROR("Can't call service set_power_off");
		return false;
	}
	return true;
}

bool ClopemaMove::getRobotState(arm_navigation_msgs::RobotState & rs) {
	ros::service::waitForService("/environment_server/get_robot_state");
	arm_navigation_msgs::GetRobotState r;
	if (!ros::service::call("/environment_server/get_robot_state", r)) {
		ROS_ERROR("Can't get current robot state.");
		return false;
	}
	rs = r.response.robot_state;
	return true;
}

bool ClopemaMove::plan(clopema_arm_navigation::ClopemaMotionPlan& mp) {
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

std::vector<std::string> ClopemaMove::getJointsInGroup(std::string group) {
	planning_environment::CollisionModels cm("robot_description");
	return cm.getKinematicModel()->getModelGroup(group)->getJointModelNames();
}

void ClopemaMove::poseToClopemaMotionPlan(clopema_arm_navigation::ClopemaMotionPlan& mp, arm_navigation_msgs::SimplePoseConstraint & desired_pose) {
	arm_navigation_msgs::PositionConstraint position_constraint;
	arm_navigation_msgs::OrientationConstraint orientation_constraint;
	arm_navigation_msgs::poseConstraintToPositionOrientationConstraints(desired_pose, position_constraint, orientation_constraint);
	mp.request.motion_plan_req.goal_constraints.orientation_constraints.push_back(orientation_constraint);
	mp.request.motion_plan_req.goal_constraints.position_constraints.push_back(position_constraint);

}

