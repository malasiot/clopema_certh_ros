#include "clopema_motoros/ClopemaMove.h"

using namespace std;

int main(int argc, char **argv) {
	ros::init(argc, argv, "move_joint_dual");
	ros::NodeHandle nh;

	ClopemaMove cmove;

	//Create plan
	clopema_arm_navigation::ClopemaMotionPlan mp;
	mp.request.motion_plan_req.group_name = "ext_axis";
	mp.request.motion_plan_req.allowed_planning_time = ros::Duration(2.0);

	//Set start state
	if (!cmove.getRobotState(mp.request.motion_plan_req.start_state))
		return -1;

	//Get all joint in the group
	vector<string> joint_names = cmove.getJointsInGroup(mp.request.motion_plan_req.group_name);
	ros::Duration(1.0).sleep();
	//Set joints goal constraints
	mp.request.motion_plan_req.goal_constraints.joint_constraints.resize(joint_names.size());
	for (unsigned int i = 0; i < mp.request.motion_plan_req.goal_constraints.joint_constraints.size(); ++i) {
		mp.request.motion_plan_req.goal_constraints.joint_constraints[i].joint_name = joint_names[i];
		mp.request.motion_plan_req.goal_constraints.joint_constraints[i].position = 0.0;
		mp.request.motion_plan_req.goal_constraints.joint_constraints[i].tolerance_below = 0.1;
		mp.request.motion_plan_req.goal_constraints.joint_constraints[i].tolerance_above = 0.1;
	}

	double desPos = 0;
	if (argc > 1) {
		std::stringstream argv1(argv[1]);
		desPos = atof(argv1.str().data());
	}
	vector<string>::iterator posIt;
	if ((posIt = find(joint_names.begin(), joint_names.end(), "ext_axis")) != joint_names.end()) {
		mp.request.motion_plan_req.goal_constraints.joint_constraints.at(std::distance(joint_names.begin(), posIt)).position = desPos;
	}

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
