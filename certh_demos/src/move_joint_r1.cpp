#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <clopema_arm_navigation/ClopemaMotionPlan.h>
#include <arm_navigation_msgs/GetRobotState.h>
#include <arm_navigation_msgs/convert_messages.h>
#include <clopema_motoros/SetPowerOff.h>

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

arm_navigation_msgs::MotionPlanRequest createPlan(double sAxis) {
	arm_navigation_msgs::MotionPlanRequest req;
	req.group_name = "r1_arm";
	req.allowed_planning_time = ros::Duration(5.0);
	std::vector<std::string> names;
	names.push_back("r1_joint_s");
	names.push_back("r1_joint_l");
	names.push_back("r1_joint_u");
	names.push_back("r1_joint_r");
	names.push_back("r1_joint_b");
	names.push_back("r1_joint_t");
	req.goal_constraints.joint_constraints.resize(names.size());
	for (unsigned int i = 0; i < req.goal_constraints.joint_constraints.size(); ++i) {
		req.goal_constraints.joint_constraints[i].joint_name = names[i];
		req.goal_constraints.joint_constraints[i].position = 0.0;
		req.goal_constraints.joint_constraints[i].tolerance_below = 0.1;
		req.goal_constraints.joint_constraints[i].tolerance_above = 0.1;
	}

	req.goal_constraints.joint_constraints[0].position = sAxis;

	return req;
}

int main(int argc, char **argv) {
	if (argc < 2) {
		ROS_ERROR("Not enough arguments: send_traj <R1sAxisDesiredPosition>");
		return -1;
	}
	ros::init(argc, argv, "send_traj");
	ros::NodeHandle nh;

	clopema_arm_navigation::ClopemaMotionPlan mp;
	std::stringstream argv1(argv[1]);
	mp.request.motion_plan_req = createPlan(atof(argv1.str().data()));

	if (!getRobotState(mp.request.motion_plan_req.start_state)) {
		ROS_ERROR("Can't get current robot state");
		return -1;
	}

	ros::service::waitForService("/clopema_planner/plan");
	if (!ros::service::call("/clopema_planner/plan", mp)) {
		ROS_ERROR("Can't call service clopema_planner/plan");
		return -1;
	}

	if (mp.response.error_code.val == arm_navigation_msgs::ArmNavigationErrorCodes::SUCCESS) {
		actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> move("/clopema_controller/follow_joint_trajectory", true);
		move.waitForServer();
		control_msgs::FollowJointTrajectoryGoal goal;
		goal.trajectory = mp.response.joint_trajectory;
		move.sendGoal(goal);
		bool finished_within_time = move.waitForResult(ros::Duration(45.0));
		if (!finished_within_time) {
			move.cancelGoal();
			ROS_INFO("Timed out achieving goal");
		} else {
			actionlib::SimpleClientGoalState state = move.getState();
			bool success = (state == actionlib::SimpleClientGoalState::SUCCEEDED);
			if (success)
				ROS_INFO("Action finished: %s", state.toString().c_str());
			else {
				ROS_INFO("Action failed: %s", state.toString().c_str());
				ROS_WARN("Addition information: %s", state.text_.c_str());
				control_msgs::FollowJointTrajectoryResult r;
				ROS_WARN("Error code: %d", move.getResult()->error_code);
			}
		}
	} else {
		ROS_WARN("Can't plan trajectory: %s", arm_navigation_msgs::armNavigationErrorCodeToString(mp.response.error_code).data());
	}

//Set servo power off
	clopema_motoros::SetPowerOff soff;
	soff.request.force = false;
	ros::service::waitForService("/joint_trajectory_action/set_power_off");
	if (!ros::service::call("/joint_trajectory_action/set_power_off", soff)) {
		ROS_ERROR("Can't call service set_power_off");
		return -1;
	}
	return 1;
}
