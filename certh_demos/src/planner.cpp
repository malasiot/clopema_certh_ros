#include <clopema_motoros/ClopemaMove.h>

using namespace std;

double fRand(double fMin, double fMax) {
	double f = (double) rand() / RAND_MAX;
	return fMin + f * (fMax - fMin);
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "benchmark_planner");
	ros::NodeHandle nh;

	srand((unsigned) time(0));

	ClopemaMove cmove;

	//Create plan
	clopema_arm_navigation::ClopemaMotionPlan mp;
	mp.request.motion_plan_req.group_name = "r2_arm";
	mp.request.motion_plan_req.allowed_planning_time = ros::Duration(5.0);
	if (!cmove.getRobotState(mp.request.motion_plan_req.start_state))
		return false;

	arm_navigation_msgs::SimplePoseConstraint desired_pose;
	desired_pose.header.frame_id = "base_link";
	desired_pose.header.stamp = ros::Time::now();
	desired_pose.link_name = "r2_tip_link";
	desired_pose.pose.position.x = 0.5;
	desired_pose.pose.position.y = -1;
	desired_pose.pose.position.z = 1.8;
	desired_pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, M_PI);
	desired_pose.absolute_position_tolerance.x = 0.02;
	desired_pose.absolute_position_tolerance.y = 0.02;
	desired_pose.absolute_position_tolerance.z = 0.02;
	desired_pose.absolute_roll_tolerance = 0.04;
	desired_pose.absolute_pitch_tolerance = 0.04;
	desired_pose.absolute_yaw_tolerance = 0.04;

	cmove.poseToClopemaMotionPlan(mp, desired_pose);
	/*desired_pose.link_name = "r1_tip_link";
	desired_pose.pose.position.x = -0.5;
	cmove.poseToClopemaMotionPlan(mp, desired_pose);*/

	ROS_INFO("Planning to 1.8 height");
	if (!cmove.plan(mp))
		return false;
	trajectory_msgs::JointTrajectory wholeTraj = mp.response.joint_trajectory;
	double allTime = 0;
	int numOfPlan = 0;
	for (double i = 0; i < 500; i++) {
		//remove goal constraints and set new one
		ROS_INFO("Total size: %d", (int)wholeTraj.points.size());
		mp.request.motion_plan_req.goal_constraints.orientation_constraints.clear();
		mp.request.motion_plan_req.goal_constraints.position_constraints.clear();
		desired_pose.pose.position.x = fRand(0.0, 1.0);
		desired_pose.pose.position.y = fRand(0.0, -1.5);
		desired_pose.pose.position.z = fRand(0.4, 1.8);
		desired_pose.link_name = "r2_tip_link";
		cmove.poseToClopemaMotionPlan(mp, desired_pose);
	/*	desired_pose.link_name = "r1_tip_link";
		desired_pose.pose.position.x = -0.5;
		cmove.poseToClopemaMotionPlan(mp, desired_pose);*/

		cmove.pointToRobotState(mp.request.motion_plan_req.start_state, wholeTraj.points.back(), wholeTraj.joint_names);

		//ros::Duration(0.05).sleep();
		ros::Time start = ros::Time::now();
		if (!cmove.plan(mp)) {
			continue;
		}

		if (!std::equal(wholeTraj.points.back().positions.begin(), wholeTraj.points.back().positions.end(), mp.response.joint_trajectory.points.at(0).positions.begin())) {
			ROS_ERROR("Not equal point");
			ROS_ERROR_STREAM(wholeTraj.points.back());
			ROS_WARN_STREAM(mp.request.motion_plan_req);
			ROS_INFO_STREAM(mp.response.joint_trajectory.points.at(0));
			return 1;
		}

		ROS_WARN("Planning time (%f,%f,%f) size: %d: %f", desired_pose.pose.position.x, desired_pose.pose.position.y, desired_pose.pose.position.z, mp.response.joint_trajectory.points.size(), (ros::Time::now().toSec() - start.toSec()));
		allTime += (ros::Time::now().toSec() - start.toSec());
		numOfPlan++;
		wholeTraj.points.insert(wholeTraj.points.end(), mp.response.joint_trajectory.points.begin(), mp.response.joint_trajectory.points.end());
	}
	ROS_INFO("Number of successful plan %d : computation time %f: avg: %f", numOfPlan, allTime, allTime/numOfPlan);

	mp.response.joint_trajectory = wholeTraj;

	control_msgs::FollowJointTrajectoryGoal goal;
	goal.trajectory = mp.response.joint_trajectory;
	cmove.cmc_.sendGoal(goal);

	return 1;
}
