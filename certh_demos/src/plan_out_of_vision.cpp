#include "ros/ros.h"
#include "clopema_arm_navigation/ClopemaPlanOutOfVision.h"
#include "std_srvs/Empty.h"
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>

using namespace std;
using namespace visualization_msgs;

boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server;
interactive_markers::MenuHandler menu_handler;

void makeMenuMarker() {
	InteractiveMarker int_marker;
	int_marker.header.frame_id = "/base_link";
	int_marker.pose.position.z = 2.5;
	int_marker.scale = 1;
	int_marker.name = "context_menu";
	int_marker.description = "Vision Menu";
	InteractiveMarkerControl control;
	control.interaction_mode = InteractiveMarkerControl::MENU;
	control.description = "Options";
	control.name = "menu_only_control";

	Marker marker;
	marker.type = Marker::SPHERE;
	marker.scale.x = 0.45;
	marker.scale.y = 0.45;
	marker.scale.z = 0.45;
	marker.color.r = 0.0;
	marker.color.g = 0.0;
	marker.color.b = 1.0;
	marker.color.a = 1.0;

	control.markers.push_back(marker);
	control.always_visible = true;
	int_marker.controls.push_back(control);

	server->insert(int_marker);
	menu_handler.apply(*server, int_marker.name);
}

void visualize(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback) {
	std_srvs::Empty em;
	ros::service::call("/plan_out_of_vision/visualize_vision", em);
}

void plan(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback) {
	actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> move("/clopema_controller/follow_joint_trajectory", true);
	move.waitForServer();
	clopema_arm_navigation::ClopemaPlanOutOfVision p;
	p.request.r1_arm = true;
	p.request.r2_arm = true;
	p.request.r1_kinect = true;
	p.request.r2_kinect = true;

	if (!ros::service::call("/plan_out_of_vision/plan", p)) {
		ROS_ERROR("Can't call plan_out_of_vision service");
		return ;
	}

	if (p.response.error_code.val != p.response.error_code.SUCCESS) {
		ROS_ERROR_STREAM("Can't plan out of vision, error code: " << p.response.error_code.val);
		return ;
	} else {
		ROS_INFO_STREAM("Successfully planned");
	}
	//Create goal and send it to the controller
	control_msgs::FollowJointTrajectoryGoal goal;
	goal.trajectory = p.response.joint_trajectory;
	move.sendGoal(goal);

	bool finished_within_time = false;
	while (!finished_within_time) {
		finished_within_time = move.waitForResult(ros::Duration(0.001));
		std_srvs::Empty em;
		ros::service::call("/plan_out_of_vision/visualize_vision", em);
	}

	if (!finished_within_time) {
		move.cancelGoal();
		ROS_INFO("Timed out achieving goal A");
	} else {
		actionlib::SimpleClientGoalState state = move.getState();
		bool success = (state == actionlib::SimpleClientGoalState::SUCCEEDED);
		if (success)
			ROS_INFO("Action finished: %s", state.toString().c_str());
		else {
			ROS_INFO("Action failed: %s", state.toString().c_str());
			ROS_WARN("Addition information: %s", state.text_.c_str());
		}
	}

}

void planR1(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback) {
	actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> move("/clopema_controller/follow_joint_trajectory", true);
	move.waitForServer();
	clopema_arm_navigation::ClopemaPlanOutOfVision p;
	p.request.r1_arm = true;
	p.request.r2_arm = false;
	p.request.r1_kinect = true;
	p.request.r2_kinect = true;

	if (!ros::service::call("/plan_out_of_vision/plan", p)) {
		ROS_ERROR("Can't call plan_out_of_vision service");
		return ;
	}

	if (p.response.error_code.val != p.response.error_code.SUCCESS) {
		ROS_ERROR_STREAM("Can't plan out of vision, error code: " << p.response.error_code.val);
		return ;
	} else {
		ROS_INFO_STREAM("Successfully planned");
	}
	//Create goal and send it to the controller
	control_msgs::FollowJointTrajectoryGoal goal;
	goal.trajectory = p.response.joint_trajectory;
	move.sendGoal(goal);

	bool finished_within_time = false;
	while (!finished_within_time) {
		finished_within_time = move.waitForResult(ros::Duration(0.001));
		std_srvs::Empty em;
		ros::service::call("/plan_out_of_vision/visualize_vision", em);
	}

	if (!finished_within_time) {
		move.cancelGoal();
		ROS_INFO("Timed out achieving goal A");
	} else {
		actionlib::SimpleClientGoalState state = move.getState();
		bool success = (state == actionlib::SimpleClientGoalState::SUCCEEDED);
		if (success)
			ROS_INFO("Action finished: %s", state.toString().c_str());
		else {
			ROS_INFO("Action failed: %s", state.toString().c_str());
			ROS_WARN("Addition information: %s", state.text_.c_str());
		}
	}

}
void planR2(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback) {
	actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> move("/clopema_controller/follow_joint_trajectory", true);
	move.waitForServer();
	clopema_arm_navigation::ClopemaPlanOutOfVision p;
	p.request.r1_arm = false;
	p.request.r2_arm = true;
	p.request.r1_kinect = true;
	p.request.r2_kinect = true;

	if (!ros::service::call("/plan_out_of_vision/plan", p)) {
		ROS_ERROR("Can't call plan_out_of_vision service");
		return ;
	}

	if (p.response.error_code.val != p.response.error_code.SUCCESS) {
		ROS_ERROR_STREAM("Can't plan out of vision, error code: " << p.response.error_code.val);
		return ;
	} else {
		ROS_INFO_STREAM("Successfully planned");
	}
	//Create goal and send it to the controller
	control_msgs::FollowJointTrajectoryGoal goal;
	goal.trajectory = p.response.joint_trajectory;
	move.sendGoal(goal);

	bool finished_within_time = false;
	while (!finished_within_time) {
		finished_within_time = move.waitForResult(ros::Duration(0.001));
		std_srvs::Empty em;
		ros::service::call("/plan_out_of_vision/visualize_vision", em);
	}

	if (!finished_within_time) {
		move.cancelGoal();
		ROS_INFO("Timed out achieving goal A");
	} else {
		actionlib::SimpleClientGoalState state = move.getState();
		bool success = (state == actionlib::SimpleClientGoalState::SUCCEEDED);
		if (success)
			ROS_INFO("Action finished: %s", state.toString().c_str());
		else {
			ROS_INFO("Action failed: %s", state.toString().c_str());
			ROS_WARN("Addition information: %s", state.text_.c_str());
		}
	}

}
int main(int argc, char **argv) {
	ros::init(argc, argv, "plan_out_of_vision_tutorial");
	ros::NodeHandle nh;

	server.reset(new interactive_markers::InteractiveMarkerServer("vision_controls", "", false));
	menu_handler.insert("Visualize vision", &visualize);
	menu_handler.insert("Go out of vision - r1_arm", &planR1);
	menu_handler.insert("Go out of vision - r2_arm", &planR2);
	menu_handler.insert("Go out of vision - both", &plan);
	makeMenuMarker();

	server->applyChanges();
	ros::spin();
	server.reset();
	return 1;
}
