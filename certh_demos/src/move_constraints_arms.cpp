#include <clopema_motoros/ClopemaMove.h>

ClopemaMove *cmove;
ros::Publisher *marker_pub;

bool planTrajectory(clopema_arm_navigation::ClopemaMotionPlan & mp, ClopemaMove & cmove);
void doneCb(const actionlib::SimpleClientGoalState& state, const control_msgs::FollowJointTrajectoryResultConstPtr & result);

void setPathConstraints(clopema_arm_navigation::ClopemaMotionPlan & mp);
bool planToStart(clopema_arm_navigation::ClopemaMotionPlan & mp, ClopemaMove & cmove);
void displayPathConstraints(clopema_arm_navigation::ClopemaMotionPlan & mp);

int main(int argc, char **argv) {
	ros::init(argc, argv, "move_constraints_arms");
	ros::NodeHandle n;
	marker_pub = new ros::Publisher;
	*marker_pub = n.advertise<visualization_msgs::Marker>("/visualization_marker", 0);

	cmove = new ClopemaMove;
	clopema_arm_navigation::ClopemaMotionPlan mp;

	/*-----------------------
	 * Go to the start position
	 * ----------------------
	 */
	if (!planToStart(mp, *cmove)) {
		ROS_ERROR("Can't plan trajectory: %s", arm_navigation_msgs::armNavigationErrorCodeToString(mp.response.error_code).data());
		return -1;
	}

	ROS_INFO("Executing trajectory size: %d", (int)mp.response.joint_trajectory.points.size());
	control_msgs::FollowJointTrajectoryGoal goal;
	goal.trajectory = mp.response.joint_trajectory;
	ROS_INFO("Sending trajectory size: %d", (int)goal.trajectory.points.size());
	cmove->cmc_.sendGoal(goal, &doneCb);
	cmove->cmc_.waitForResult(ros::Duration(60));

	mp.request.motion_plan_req.goal_constraints.joint_constraints.clear();
	mp.request.motion_plan_req.goal_constraints.position_constraints.clear();
	mp.request.motion_plan_req.goal_constraints.orientation_constraints.clear();
	/**
	 * Plan with constraints
	 */

	setPathConstraints(mp);
	displayPathConstraints(mp);

	if (!planTrajectory(mp, *cmove)) {
		ROS_ERROR("Can't plan trajectory: %s", arm_navigation_msgs::armNavigationErrorCodeToString(mp.response.error_code).data());
		return -1;
	}

	ROS_INFO("Executing trajectory size: %d", (int)mp.response.joint_trajectory.points.size());
	goal.trajectory = mp.response.joint_trajectory;

	ROS_INFO("Sending trajectory size: %d", (int)goal.trajectory.points.size());
	cmove->cmc_.sendGoal(goal, &doneCb);

	cmove->cmc_.waitForResult(ros::Duration(60));

	delete cmove;
	delete marker_pub;
	return 1;
}

void displayPathConstraints(clopema_arm_navigation::ClopemaMotionPlan & mp) {
	for (int i = 0; i < (int) mp.request.motion_plan_req.path_constraints.position_constraints.size(); i++) {
		visualization_msgs::Marker marker;
		marker.header.frame_id = "base_link";
		marker.header.stamp = ros::Time::now();
		marker.ns = "path_constraints";
		marker.id = i;
		marker.type = visualization_msgs::Marker::CUBE;
		marker.action = visualization_msgs::Marker::ADD;
		marker.pose.position.x = mp.request.motion_plan_req.path_constraints.position_constraints.at(i).position.x;
		marker.pose.position.y = mp.request.motion_plan_req.path_constraints.position_constraints.at(i).position.y;
		marker.pose.position.z = mp.request.motion_plan_req.path_constraints.position_constraints.at(i).position.z;
		marker.pose.orientation.x = mp.request.motion_plan_req.path_constraints.position_constraints.at(i).constraint_region_orientation.x;
		marker.pose.orientation.y = mp.request.motion_plan_req.path_constraints.position_constraints.at(i).constraint_region_orientation.y;
		marker.pose.orientation.z = mp.request.motion_plan_req.path_constraints.position_constraints.at(i).constraint_region_orientation.z;
		marker.pose.orientation.w = mp.request.motion_plan_req.path_constraints.position_constraints.at(i).constraint_region_orientation.w;
		marker.scale.x = mp.request.motion_plan_req.path_constraints.position_constraints.at(i).constraint_region_shape.dimensions.at(0);
		marker.scale.y = mp.request.motion_plan_req.path_constraints.position_constraints.at(i).constraint_region_shape.dimensions.at(1);
		marker.scale.z = mp.request.motion_plan_req.path_constraints.position_constraints.at(i).constraint_region_shape.dimensions.at(2);

		marker.color.r = 0.0f;
		marker.color.g = 1.0f;
		marker.color.b = i / mp.request.motion_plan_req.path_constraints.position_constraints.size();
		marker.color.a = 0.5;

		marker.lifetime = ros::Duration();

		ros::Duration(0.3).sleep();
		marker_pub->publish(marker);
		ros::Duration(0.3).sleep();
	}
}
void setPathConstraints(clopema_arm_navigation::ClopemaMotionPlan & mp) {/*
 mp.request.motion_plan_req.path_constraints.orientation_constraints.resize(1);
 mp.request.motion_plan_req.path_constraints.orientation_constraints[0].header.frame_id = "base_link";
 mp.request.motion_plan_req.path_constraints.orientation_constraints[0].header.stamp = ros::Time::now();
 mp.request.motion_plan_req.path_constraints.orientation_constraints[0].link_name = "r2_tip_link";
 mp.request.motion_plan_req.path_constraints.orientation_constraints[0].orientation.x = 0.0;
 mp.request.motion_plan_req.path_constraints.orientation_constraints[0].orientation.y = 0.0;
 mp.request.motion_plan_req.path_constraints.orientation_constraints[0].orientation.z = 0.0;
 mp.request.motion_plan_req.path_constraints.orientation_constraints[0].orientation.w = 1.0;
 mp.request.motion_plan_req.path_constraints.orientation_constraints[0].type = arm_navigation_msgs::OrientationConstraint::HEADER_FRAME;
 mp.request.motion_plan_req.path_constraints.orientation_constraints[0].absolute_roll_tolerance = 0.4;
 mp.request.motion_plan_req.path_constraints.orientation_constraints[0].absolute_pitch_tolerance = 0.4;
 mp.request.motion_plan_req.path_constraints.orientation_constraints[0].absolute_yaw_tolerance = 0.4;*/

	mp.request.motion_plan_req.path_constraints.position_constraints.resize(2);
	mp.request.motion_plan_req.path_constraints.position_constraints[0].header.frame_id = "base_link";
	mp.request.motion_plan_req.path_constraints.position_constraints[0].header.stamp = ros::Time::now();
	mp.request.motion_plan_req.path_constraints.position_constraints[0].link_name = "r2_tip_link";
	mp.request.motion_plan_req.path_constraints.position_constraints[0].position.x = 0.5;
	mp.request.motion_plan_req.path_constraints.position_constraints[0].position.y = -1.0;
	mp.request.motion_plan_req.path_constraints.position_constraints[0].position.z = 1.0;
	mp.request.motion_plan_req.path_constraints.position_constraints[0].constraint_region_shape.type = arm_navigation_msgs::Shape::BOX;
	mp.request.motion_plan_req.path_constraints.position_constraints[0].constraint_region_shape.dimensions.push_back(1.1);
	mp.request.motion_plan_req.path_constraints.position_constraints[0].constraint_region_shape.dimensions.push_back(0.1);
	mp.request.motion_plan_req.path_constraints.position_constraints[0].constraint_region_shape.dimensions.push_back(0.1);
	mp.request.motion_plan_req.path_constraints.position_constraints[0].constraint_region_orientation.x = 0.0;
	mp.request.motion_plan_req.path_constraints.position_constraints[0].constraint_region_orientation.y = 0.0;
	mp.request.motion_plan_req.path_constraints.position_constraints[0].constraint_region_orientation.z = 0.0;
	mp.request.motion_plan_req.path_constraints.position_constraints[0].constraint_region_orientation.w = 1.0;
	mp.request.motion_plan_req.path_constraints.position_constraints[0].weight = 1.0;
	mp.request.motion_plan_req.path_constraints.position_constraints[1] = mp.request.motion_plan_req.path_constraints.position_constraints[0];
	mp.request.motion_plan_req.path_constraints.position_constraints[1].link_name = "r1_tip_link";
	mp.request.motion_plan_req.path_constraints.position_constraints[1].position.x = 0.4;

}

void doneCb(const actionlib::SimpleClientGoalState& state, const control_msgs::FollowJointTrajectoryResultConstPtr & result) {
	ROS_INFO("Finished in state [%s:%s]", state.toString().c_str(), state.text_.data());
	ROS_INFO("Result - error code: %d", result->error_code);
	cmove->setServoPowerOff(false);
}

bool planToStart(clopema_arm_navigation::ClopemaMotionPlan & mp, ClopemaMove & cmove) {
	mp.request.motion_plan_req.group_name = "arms";
	mp.request.motion_plan_req.allowed_planning_time = ros::Duration(15.0);
	if (!cmove.getRobotState(mp.request.motion_plan_req.start_state))
		return false;

	arm_navigation_msgs::SimplePoseConstraint desired_pose;
	desired_pose.header.frame_id = "base_link";
	desired_pose.header.stamp = ros::Time::now();
	desired_pose.link_name = "r2_tip_link";
	desired_pose.pose.position.x = 1.0;
	desired_pose.pose.position.y = -1;
	desired_pose.pose.position.z = 1.0;
	desired_pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(M_PI, 0, 0);
	desired_pose.absolute_position_tolerance.x = 0.02;
	desired_pose.absolute_position_tolerance.y = 0.02;
	desired_pose.absolute_position_tolerance.z = 0.02;
	desired_pose.absolute_roll_tolerance = 0.04;
	desired_pose.absolute_pitch_tolerance = 0.04;
	desired_pose.absolute_yaw_tolerance = 0.04;

	cmove.poseToClopemaMotionPlan(mp, desired_pose);
	desired_pose.link_name = "r1_tip_link";
	desired_pose.pose.position.x -= 0.5;
	cmove.poseToClopemaMotionPlan(mp, desired_pose);

	ROS_INFO("Planning to 1.0 height");
	if (!cmove.plan(mp))
		return false;
	trajectory_msgs::JointTrajectory wholeTraj = mp.response.joint_trajectory;
	mp.response.joint_trajectory = wholeTraj;
	return true;
}

bool planTrajectory(clopema_arm_navigation::ClopemaMotionPlan & mp, ClopemaMove & cmove) {
	mp.request.motion_plan_req.group_name = "arms";
	mp.request.motion_plan_req.allowed_planning_time = ros::Duration(60.0);
	if (!cmove.getRobotState(mp.request.motion_plan_req.start_state))
		return false;

	arm_navigation_msgs::SimplePoseConstraint desired_pose;
	desired_pose.header.frame_id = "base_link";
	desired_pose.header.stamp = ros::Time::now();
	desired_pose.link_name = "r2_tip_link";
	desired_pose.pose.position.x = 1.0;
	desired_pose.pose.position.y = -1;
	desired_pose.pose.position.z = 1.0;
	desired_pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(M_PI, 0, 0);
	desired_pose.absolute_position_tolerance.x = 0.02;
	desired_pose.absolute_position_tolerance.y = 0.02;
	desired_pose.absolute_position_tolerance.z = 0.02;
	desired_pose.absolute_roll_tolerance = 0.04;
	desired_pose.absolute_pitch_tolerance = 0.04;
	desired_pose.absolute_yaw_tolerance = 0.04;

	cmove.poseToClopemaMotionPlan(mp, desired_pose);
	desired_pose.link_name = "r1_tip_link";
	desired_pose.pose.position.x -= 0.5;
	cmove.poseToClopemaMotionPlan(mp, desired_pose);
	ROS_INFO("Planning to 1.0");
	if (!cmove.plan(mp))
		return false;
	trajectory_msgs::JointTrajectory wholeTraj = mp.response.joint_trajectory;

	for (double i = 1.0; i > 0.0; i -= 0.05) {
		//double i = 0.0;
		//remove goal constraints and set new one
		mp.request.motion_plan_req.goal_constraints.orientation_constraints.clear();
		mp.request.motion_plan_req.goal_constraints.position_constraints.clear();
		desired_pose.pose.position.x = i;
		desired_pose.link_name = "r2_tip_link";
		cmove.poseToClopemaMotionPlan(mp, desired_pose);
		desired_pose.link_name = "r1_tip_link";
		desired_pose.pose.position.x -= 0.5;
		cmove.poseToClopemaMotionPlan(mp, desired_pose);
		cmove.pointToRobotState(mp.request.motion_plan_req.start_state, wholeTraj.points.back(), wholeTraj.joint_names);

		ROS_INFO("Planning to %f. ", i);
		if (!cmove.plan(mp)) {
			mp.response.joint_trajectory = wholeTraj;
			return true;
		}
		wholeTraj.points.insert(wholeTraj.points.end(), mp.response.joint_trajectory.points.begin(), mp.response.joint_trajectory.points.end());
	}

	mp.response.joint_trajectory = wholeTraj;
	return true;
}
