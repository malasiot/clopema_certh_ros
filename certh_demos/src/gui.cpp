#include <clopema_motoros/ClopemaMove.h>

#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>
#include <arm_navigation_msgs/GetPlanningScene.h>
#include <planning_environment/models/collision_models.h>
#include <planning_environment/models/model_utils.h>
#include <kinematics_msgs/GetConstraintAwarePositionIK.h>

using namespace visualization_msgs;

ClopemaMove *cmove_ = NULL;
planning_environment::CollisionModels* cm_ = NULL;
planning_models::KinematicState* state_ = NULL;
boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server;
interactive_markers::MenuHandler menu_handler;
ros::Publisher vis_marker_array_publisher_;

trajectory_msgs::JointTrajectory last_trajectory;

int stop(int i) {
	cm_->revertPlanningScene(state_);
	delete cm_;
	delete cmove_;
	return i;
}

void setPlanningScene() {
	//----------------Setting planning scene-----------------
	ros::service::waitForService("/environment_server/set_planning_scene_diff");
	arm_navigation_msgs::GetPlanningScene planning_scene;
	if (!ros::service::call("/environment_server/set_planning_scene_diff", planning_scene)) {
		ROS_WARN("Can't get planning scene");
		stop(-1);
	}
	cm_ = new planning_environment::CollisionModels("robot_description");
	state_ = cm_->setPlanningScene(planning_scene.response.planning_scene);
	//----------------Setting planning scene END-----------
}

void publishRobotState() {
	std::vector<std::string> arm_names;
	std::vector<std::string> arm_names2;
	arm_names = cm_->getKinematicModel()->getModelGroup("r1_arm")->getUpdatedLinkModelNames();
	arm_names2 = cm_->getKinematicModel()->getModelGroup("r2_arm")->getUpdatedLinkModelNames();
	arm_names.insert(arm_names.begin(), arm_names2.begin(), arm_names2.end());
	arm_names.push_back("r1_gripper");
	arm_names.push_back("r2_gripper");
	arm_names.push_back("r750");

	std_msgs::ColorRGBA color;
	visualization_msgs::MarkerArray arr;
	color.a = color.b = color.r = color.g = 0.8;
	cm_->getRobotMarkersGivenState(*state_, arr, color, "Robot_marker", ros::Duration(), &arm_names);

	vis_marker_array_publisher_.publish(arr);
}

bool IKR1(geometry_msgs::PoseStamped pose, sensor_msgs::JointState &solution) {
	kinematics_msgs::GetConstraintAwarePositionIK ik;
	ik.request.ik_request.pose_stamped = pose;
	planning_environment::convertKinematicStateToRobotState(*state_, ros::Time::now(), cm_->getWorldFrameId(), ik.request.ik_request.robot_state);
	ik.request.ik_request.ik_seed_state = ik.request.ik_request.robot_state;
	ik.request.ik_request.ik_link_name = "r1_tip_link";
	ik.request.timeout = ros::Duration(2.0);
	ros::service::waitForService("/clopema_r1_arm_kinematics/get_constraint_aware_ik");
	if (ros::service::call("/clopema_r1_arm_kinematics/get_constraint_aware_ik", ik)) {
		if (ik.response.error_code.val != ik.response.error_code.SUCCESS) {
			ROS_WARN("IK solution not found, error code: %d", ik.response.error_code.val);
			return false;
		} else {
			solution = ik.response.solution.joint_state;
			return true;
		}
	} else {
		ROS_ERROR("Can't call IKR1 service.");
		return false;
	}
}

bool IKR2(geometry_msgs::PoseStamped pose, sensor_msgs::JointState &solution) {
	kinematics_msgs::GetConstraintAwarePositionIK ik;
	ik.request.ik_request.pose_stamped = pose;
	planning_environment::convertKinematicStateToRobotState(*state_, ros::Time::now(), cm_->getWorldFrameId(), ik.request.ik_request.robot_state);
	ik.request.ik_request.ik_seed_state = ik.request.ik_request.robot_state;
	ik.request.ik_request.ik_link_name = "r2_tip_link";
	ik.request.timeout = ros::Duration(2.0);
	ros::service::waitForService("/clopema_r2_arm_kinematics/get_constraint_aware_ik");
	if (ros::service::call("/clopema_r2_arm_kinematics/get_constraint_aware_ik", ik)) {
		if (ik.response.error_code.val != ik.response.error_code.SUCCESS) {
			ROS_WARN("IK solution not found, error code: %d", ik.response.error_code.val);
			return false;
		} else {
			solution = ik.response.solution.joint_state;
			return true;
		}
	} else {
		ROS_ERROR("Can't call IKR2 service.");
		return false;
	}
}

void r1Feedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback) {
	//ROS_INFO_STREAM(feedback->pose.position.x<<feedback->pose.position.y<<feedback->pose.position.z);
	geometry_msgs::PoseStamped pose;
	pose.header.frame_id = "base_link";
	pose.pose.position = feedback->pose.position;
	pose.pose.orientation = feedback->pose.orientation;

	sensor_msgs::JointState sol;
	if (!IKR1(pose, sol))
		return;

	std::map<std::string, double> nval;
	for (int i = 0; i < (int) sol.name.size(); i++) {
		nval[sol.name.at(i)] = sol.position.at(i);
	}
	state_->setKinematicState(nval);
	publishRobotState();
}
void r2Feedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback) {
	//ROS_INFO_STREAM(feedback->pose.position.x<<feedback->pose.position.y<<feedback->pose.position.z);
	geometry_msgs::PoseStamped pose;
	pose.header.frame_id = "base_link";
	pose.pose.position = feedback->pose.position;
	pose.pose.orientation = feedback->pose.orientation;

	sensor_msgs::JointState sol;
	if (!IKR2(pose, sol))
		return;

	std::map<std::string, double> nval;
	for (int i = 0; i < (int) sol.name.size(); i++) {
		nval[sol.name.at(i)] = sol.position.at(i);
	}
	state_->setKinematicState(nval);
	publishRobotState();
}

void planTrajectory(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback) {
	ROS_INFO("Planning");
	clopema_arm_navigation::ClopemaMotionPlan mp;
	mp.request.motion_plan_req.group_name = "arms";
	mp.request.motion_plan_req.allowed_planning_time = ros::Duration(5.0);
	if (!cmove_->getRobotState(mp.request.motion_plan_req.start_state))
		return;
	std::vector<std::string> joint_names = cmove_->getJointsInGroup(mp.request.motion_plan_req.group_name);

	mp.request.motion_plan_req.goal_constraints.joint_constraints.resize(joint_names.size());
	for (unsigned int i = 0; i < mp.request.motion_plan_req.goal_constraints.joint_constraints.size(); ++i) {
		mp.request.motion_plan_req.goal_constraints.joint_constraints[i].joint_name = joint_names[i];
		mp.request.motion_plan_req.goal_constraints.joint_constraints[i].position = state_->getJointState(joint_names[i])->getJointStateValues().at(0);
		mp.request.motion_plan_req.goal_constraints.joint_constraints[i].tolerance_below = 0.005;
		mp.request.motion_plan_req.goal_constraints.joint_constraints[i].tolerance_above = 0.005;
	}
	if (!cmove_->plan(mp))
		return;

	last_trajectory = mp.response.joint_trajectory;
	ROS_INFO("Planning completed");
}
void visualizeTrajectory(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback) {
	ROS_INFO_STREAM("Trajectory size:" << last_trajectory.points.size());
	for (int i = 0; i < (int) last_trajectory.points.size(); i++) {

		std::map<std::string, double> nval;
		for (int j = 0; j < (int) last_trajectory.joint_names.size(); j++) {
			nval[last_trajectory.joint_names.at(j)] = last_trajectory.points.at(i).positions.at(j);
		}
		state_->setKinematicState(nval);
		publishRobotState();
		ros::Duration(0.05).sleep();
	}
}
void executeTrajectory(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback) {
	ROS_INFO("Executing trajectory.");
	control_msgs::FollowJointTrajectoryGoal goal;
	goal.trajectory = last_trajectory;
	cmove_->cmc_.sendGoal(goal);
	if (!cmove_->cmc_.waitForResult(ros::Duration(60)))
		cmove_->cmc_.cancelGoal();
}
void setServoPowerOff(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback) {
	cmove_->setServoPowerOff(false);
}
void setServoPowerOffForce(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback) {
	cmove_->setServoPowerOff(true);
}

void setHomePosition(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback) {
	state_->setKinematicStateToDefault();
	state_->updateKinematicLinks();
	tf::Transform tran = state_->getLinkState("r1_tip_link")->getGlobalLinkTransform();
	geometry_msgs::Pose pose;
	pose.position.x = tran.getOrigin().x();
	pose.position.y = tran.getOrigin().y();
	pose.position.z = tran.getOrigin().z();
	pose.orientation.x = tran.getRotation().x();
	pose.orientation.y = tran.getRotation().y();
	pose.orientation.z = tran.getRotation().z();
	pose.orientation.w = tran.getRotation().w();
	server->setPose("r1", pose);

	tran = state_->getLinkState("r2_tip_link")->getGlobalLinkTransform();
	pose.position.x = tran.getOrigin().x();
	pose.position.y = tran.getOrigin().y();
	pose.position.z = tran.getOrigin().z();
	pose.orientation.x = tran.getRotation().x();
	pose.orientation.y = tran.getRotation().y();
	pose.orientation.z = tran.getRotation().z();
	pose.orientation.w = tran.getRotation().w();
	server->setPose("r2", pose);
	server->applyChanges();
	publishRobotState();
}

void planVisExeOff(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback) {
	planTrajectory(feedback);
	visualizeTrajectory(feedback);
	executeTrajectory(feedback);
	setServoPowerOff(feedback);
}

InteractiveMarkerControl& makeBoxControl(InteractiveMarker &msg) {
	visualization_msgs::Marker marker;
	marker.type = visualization_msgs::Marker::CUBE;
	marker.scale.x = 0.1;
	marker.scale.y = 0.1;
	marker.scale.z = 0.1;
	marker.color.r = 0.5;
	marker.color.g = 0.5;
	marker.color.b = 0.5;
	marker.color.a = 0.7;
	InteractiveMarkerControl control;
	control.always_visible = true;
	control.markers.push_back(marker);
	msg.controls.push_back(control);
	return msg.controls.back();
}

void makeBox(int robot) {
	InteractiveMarker int_marker;
	int_marker.header.frame_id = "/base_link";

	if (robot == 1) {
		int_marker.scale = 0.25;
		int_marker.name = "r1";
		int_marker.description = "First arm";

		state_->updateKinematicLinks();
		tf::Transform tran = state_->getLinkState("r1_tip_link")->getGlobalLinkTransform();
		int_marker.pose.position.x = tran.getOrigin().x();
		int_marker.pose.position.y = tran.getOrigin().y();
		int_marker.pose.position.z = tran.getOrigin().z();
		int_marker.pose.orientation.x = tran.getRotation().x();
		int_marker.pose.orientation.y = tran.getRotation().y();
		int_marker.pose.orientation.z = tran.getRotation().z();
		int_marker.pose.orientation.w = tran.getRotation().w();
	} else if (robot == 2) {
		int_marker.scale = 0.25;
		int_marker.name = "r2";
		int_marker.description = "Second arm";

		state_->updateKinematicLinks();
		tf::Transform tran = state_->getLinkState("r2_tip_link")->getGlobalLinkTransform();
		int_marker.pose.position.x = tran.getOrigin().x();
		int_marker.pose.position.y = tran.getOrigin().y();
		int_marker.pose.position.z = tran.getOrigin().z();
		int_marker.pose.orientation.x = tran.getRotation().x();
		int_marker.pose.orientation.y = tran.getRotation().y();
		int_marker.pose.orientation.z = tran.getRotation().z();
		int_marker.pose.orientation.w = tran.getRotation().w();
	} else {
		return;
	}
	// insert a box
	makeBoxControl(int_marker);
	InteractiveMarkerControl control;
	control.orientation_mode = InteractiveMarkerControl::FIXED;

	control.orientation.w = 1;
	control.orientation.x = 1;
	control.orientation.y = 0;
	control.orientation.z = 0;
	control.name = "rotate_x";
	control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
	int_marker.controls.push_back(control);
	control.name = "move_x";
	control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
	int_marker.controls.push_back(control);

	control.orientation.w = 1;
	control.orientation.x = 0;
	control.orientation.y = 1;
	control.orientation.z = 0;
	control.name = "rotate_z";
	control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
	int_marker.controls.push_back(control);
	control.name = "move_z";
	control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
	int_marker.controls.push_back(control);

	control.orientation.w = 1;
	control.orientation.x = 0;
	control.orientation.y = 0;
	control.orientation.z = 1;
	control.name = "rotate_y";
	control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
	int_marker.controls.push_back(control);
	control.name = "move_y";
	control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
	int_marker.controls.push_back(control);

	server->insert(int_marker);
	if (robot == 1) {
		server->setCallback(int_marker.name, &r1Feedback);
	} else if (robot == 2) {
		server->setCallback(int_marker.name, &r2Feedback);
	}
}

void makeMenuMarker() {
	InteractiveMarker int_marker;
	int_marker.header.frame_id = "/base_link";
	int_marker.pose.position.z = 3.0;
	int_marker.scale = 1;
	int_marker.name = "context_menu";
	int_marker.description = "Menu\n(Right Click on Red Sphere)";
	InteractiveMarkerControl control;
	control.interaction_mode = InteractiveMarkerControl::MENU;
	control.description = "Options";
	control.name = "menu_only_control";

	Marker marker;
	marker.type = Marker::SPHERE;
	marker.scale.x = 0.45;
	marker.scale.y = 0.45;
	marker.scale.z = 0.45;
	marker.color.r = 1.0;
	marker.color.g = 0.0;
	marker.color.b = 0.0;
	marker.color.a = 1.0;

	control.markers.push_back(marker);
	control.always_visible = true;
	int_marker.controls.push_back(control);

	server->insert(int_marker);
	//server->setCallback(int_marker.name, &processMenu);
	menu_handler.apply(*server, int_marker.name);
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "GUI_planner");
	ros::NodeHandle n;
	cmove_ = new ClopemaMove;
	setPlanningScene();
	server.reset(new interactive_markers::InteractiveMarkerServer("basic_controls", "", false));
	vis_marker_array_publisher_ = n.advertise<visualization_msgs::MarkerArray>("state_validity_markers_array", 128);

	ros::Duration(0.5).sleep();

	menu_handler.insert("Plan", &planTrajectory);
	menu_handler.insert("Visualize trajectory", &visualizeTrajectory);
	menu_handler.insert("Execute trajectory", &executeTrajectory);
	menu_handler.insert("Set to the home position", &setHomePosition);
	interactive_markers::MenuHandler::EntryHandle sub_menu_handle = menu_handler.insert("Servo Power");
	menu_handler.insert(sub_menu_handle, "Turn off", &setServoPowerOff);
	menu_handler.insert(sub_menu_handle, "Turn off - force!", &setServoPowerOffForce);
	menu_handler.insert("Plan & Visualize & Execute & Turn off", &planVisExeOff);

	makeBox(1);
	makeBox(2);
	makeMenuMarker();
	publishRobotState();

	server->applyChanges();
	ros::spin();
	server.reset();
	return stop(1);
}
