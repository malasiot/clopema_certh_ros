#include "clopema_motoros/ClopemaMove.h"

using namespace std;

int main(int argc, char **argv) {
	ros::init(argc, argv, "move_home");
	ros::NodeHandle nh;

	/* --------------------------------------------------------
	 * Set both arms to the home position using dual arm joint planner (group = arms)
	 * --------------------------------------------------------*/
	ClopemaMove cmove;
	clopema_arm_navigation::ClopemaMotionPlan mp;
	mp.request.motion_plan_req.group_name = "arms";
	mp.request.motion_plan_req.allowed_planning_time = ros::Duration(5.0);
	if (!cmove.getRobotState(mp.request.motion_plan_req.start_state))
		return -1;
	vector<string> joint_names = cmove.getJointsInGroup(mp.request.motion_plan_req.group_name);
	mp.request.motion_plan_req.goal_constraints.joint_constraints.resize(joint_names.size());
	for (unsigned int i = 0; i < mp.request.motion_plan_req.goal_constraints.joint_constraints.size(); ++i) {
		mp.request.motion_plan_req.goal_constraints.joint_constraints[i].joint_name = joint_names[i];
		mp.request.motion_plan_req.goal_constraints.joint_constraints[i].position = 0.0;
		mp.request.motion_plan_req.goal_constraints.joint_constraints[i].tolerance_below = 0.1;
		mp.request.motion_plan_req.goal_constraints.joint_constraints[i].tolerance_above = 0.1;
	}
	ROS_INFO("Planning");
	if (!cmove.plan(mp))
		return -1;
	ROS_INFO("Executing");
	control_msgs::FollowJointTrajectoryGoal goal;
	goal.trajectory = mp.response.joint_trajectory;
	cmove.doGoal(goal);

	/* --------------------------------------------------------
	 * Set ext axis to the home position
	 * --------------------------------------------------------*/
	mp.request.motion_plan_req.group_name = "ext_axis";
	mp.request.motion_plan_req.allowed_planning_time = ros::Duration(5.0);
	if (!cmove.getRobotState(mp.request.motion_plan_req.start_state))
		return -1;
	joint_names = cmove.getJointsInGroup(mp.request.motion_plan_req.group_name);
	mp.request.motion_plan_req.goal_constraints.joint_constraints.resize(joint_names.size());
	for (unsigned int i = 0; i < mp.request.motion_plan_req.goal_constraints.joint_constraints.size(); ++i) {
		mp.request.motion_plan_req.goal_constraints.joint_constraints[i].joint_name = joint_names[i];
		mp.request.motion_plan_req.goal_constraints.joint_constraints[i].position = 0.0;
		mp.request.motion_plan_req.goal_constraints.joint_constraints[i].tolerance_below = 0.1;
		mp.request.motion_plan_req.goal_constraints.joint_constraints[i].tolerance_above = 0.1;
	}
	ROS_INFO("Planning");
	if (!cmove.plan(mp))
		return -1;
	ROS_INFO("Executing");
	goal.trajectory = mp.response.joint_trajectory;
	cmove.doGoal(goal);

	ros::shutdown();
	return 1;
}
