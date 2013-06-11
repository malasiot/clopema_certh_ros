#include "clopema_motoros/ClopemaMove.h"

using namespace std;

int main(int argc, char **argv) {
	ros::init(argc, argv, "move_pose_dual");
	ros::NodeHandle nh;

	ClopemaMove cmove;

	//Create plan
	clopema_arm_navigation::ClopemaMotionPlan mp;
	mp.request.motion_plan_req.group_name = "arms";
	mp.request.motion_plan_req.allowed_planning_time = ros::Duration(5.0);

	//Set start state
	if (!cmove.getRobotState(mp.request.motion_plan_req.start_state))
		return -1;

	//Set height to desPos (minimum is 0.6)
	double desPos = 0.6;
	if (argc > 1) {
		std::stringstream argv1(argv[1]);
		desPos = atof(argv1.str().data());
	}
	if (desPos < 0.6)
		desPos = 0.6;

	arm_navigation_msgs::SimplePoseConstraint desired_pose;
	desired_pose.header.frame_id = "base_link";
	desired_pose.header.stamp = ros::Time::now();
	desired_pose.link_name = "r2_tip_link";
	desired_pose.pose.position.x = 0.5;
	desired_pose.pose.position.y = -1;
	desired_pose.pose.position.z = desPos;
	desired_pose.pose.orientation.x = 1.0;
	desired_pose.pose.orientation.y = 0.0;
	desired_pose.pose.orientation.z = 0.0;
	desired_pose.pose.orientation.w = 0.0;
	desired_pose.absolute_position_tolerance.x = 0.02;
	desired_pose.absolute_position_tolerance.y = 0.02;
	desired_pose.absolute_position_tolerance.z = 0.02;
	desired_pose.absolute_roll_tolerance = 0.04;
	desired_pose.absolute_pitch_tolerance = 0.04;
	desired_pose.absolute_yaw_tolerance = 0.04;

	arm_navigation_msgs::PositionConstraint position_constraint;
	arm_navigation_msgs::OrientationConstraint orientation_constraint;
	arm_navigation_msgs::poseConstraintToPositionOrientationConstraints(desired_pose, position_constraint, orientation_constraint);
	mp.request.motion_plan_req.goal_constraints.orientation_constraints.push_back(orientation_constraint);
	mp.request.motion_plan_req.goal_constraints.position_constraints.push_back(position_constraint);

	//add r1 goal constraints
	desired_pose.link_name = "r1_tip_link";
	desired_pose.pose.position.x = -0.5;
	arm_navigation_msgs::poseConstraintToPositionOrientationConstraints(desired_pose, position_constraint, orientation_constraint);
	mp.request.motion_plan_req.goal_constraints.orientation_constraints.push_back(orientation_constraint);
	mp.request.motion_plan_req.goal_constraints.position_constraints.push_back(position_constraint);

	ROS_INFO("Planning");
	if (!cmove.plan(mp))
		return -1;

	ROS_INFO("Executing");
	control_msgs::FollowJointTrajectoryGoal goal;
	goal.trajectory = mp.response.joint_trajectory;
	cmove.doGoal(goal);

	ros::shutdown();
	return 1;
}
