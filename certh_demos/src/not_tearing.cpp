#include <clopema_motoros/ClopemaMove.h>
#include <arm_navigation_msgs/GetPlanningScene.h>
boost::shared_ptr<ClopemaMove> cm;
boost::shared_ptr<ros::Publisher> marker_pub;

bool planTrajectory(clopema_arm_navigation::ClopemaMotionPlan & mp, ClopemaMove & cmove);
void doneCb(const actionlib::SimpleClientGoalState& state, const control_msgs::FollowJointTrajectoryResultConstPtr & result);

void setPathConstraints(clopema_arm_navigation::ClopemaMotionPlan & mp);
bool planToStart(clopema_arm_navigation::ClopemaMotionPlan & mp, ClopemaMove & cmove);
void displayPathConstraints(clopema_arm_navigation::ClopemaMotionPlan & mp);

int main(int argc, char **argv) {
	ros::init(argc, argv, "not_tearing");
	ros::NodeHandle n;
	marker_pub.reset(new ros::Publisher);
	*marker_pub = n.advertise<visualization_msgs::MarkerArray>("state_validity_markers_array", 128);
	cm.reset(new ClopemaMove);
	clopema_arm_navigation::ClopemaMotionPlan mp;

	/* ------------------------
	 * Adding collision object for purpose of presentation
	 * ------------------------
	 */
	ros::service::waitForService("/environment_server/set_planning_scene_diff");
	ros::ServiceClient get_planning_scene_client = n.serviceClient<arm_navigation_msgs::GetPlanningScene>("/environment_server/set_planning_scene_diff");
	arm_navigation_msgs::GetPlanningScene::Request planning_scene_req;
	arm_navigation_msgs::GetPlanningScene::Response planning_scene_res;

	//add the box into the collision space
	arm_navigation_msgs::CollisionObject box_object;
	box_object.id = "col_box_tearing";
	box_object.operation.operation = arm_navigation_msgs::CollisionObjectOperation::ADD;
	box_object.header.frame_id = "base_link";
	box_object.header.stamp = ros::Time::now();
	arm_navigation_msgs::Shape object;
	object.type = arm_navigation_msgs::Shape::BOX;
	object.dimensions.resize(3);
	object.dimensions[0] = .05;
	object.dimensions[1] = 0.3;
	object.dimensions[2] = 0.8;
	geometry_msgs::Pose pose;
	pose.position.x = .0;
	pose.position.y = -1.0;
	pose.position.z = 1.0;
	pose.orientation.x = 0;
	pose.orientation.y = 0;
	pose.orientation.z = 0;
	pose.orientation.w = 1;
	box_object.shapes.push_back(object);
	box_object.poses.push_back(pose);

	planning_scene_req.planning_scene_diff.collision_objects.push_back(box_object);

	if (!get_planning_scene_client.call(planning_scene_req, planning_scene_res)) {
		ROS_WARN("Can't get planning scene");
		return -1;
	}

	/*-----------------------
	 * Go to the start position
	 * ----------------------
	 */
	if (!planToStart(mp, *cm)) {
		ROS_ERROR("Can't plan trajectory: %s", arm_navigation_msgs::armNavigationErrorCodeToString(mp.response.error_code).data());
		return -1;
	}

	control_msgs::FollowJointTrajectoryGoal goal;
	if (mp.response.joint_trajectory.points.size() > 2) {
		ROS_INFO("Executing trajectory size: %d", (int)mp.response.joint_trajectory.points.size());
		goal.trajectory = mp.response.joint_trajectory;
		ROS_INFO("Sending trajectory size: %d", (int)goal.trajectory.points.size());
		cm->cmc_.sendGoal(goal, &doneCb);
		cm->cmc_.waitForResult(ros::Duration(120));
	}
	mp.request.motion_plan_req.goal_constraints.joint_constraints.clear();
	mp.request.motion_plan_req.goal_constraints.position_constraints.clear();
	mp.request.motion_plan_req.goal_constraints.orientation_constraints.clear();

	ROS_INFO_STREAM("Press enter.");
	getchar();

	/**
	 * Plan with constraints
	 */
	if (argc > 1) {
	} else {
		setPathConstraints(mp);
	}
	displayPathConstraints(mp);

	if (!planTrajectory(mp, *cm)) {
		ROS_ERROR("Can't plan trajectory: %s", arm_navigation_msgs::armNavigationErrorCodeToString(mp.response.error_code).data());
		return -1;
	}

	ROS_INFO("Executing trajectory size: %d", (int)mp.response.joint_trajectory.points.size());
	goal.trajectory = mp.response.joint_trajectory;

	ROS_INFO("Sending trajectory size: %d", (int)goal.trajectory.points.size());
	cm->cmc_.sendGoal(goal, &doneCb);

	bool finished = false;
	while (!finished) {
		finished = cm->cmc_.waitForResult(ros::Duration(0.001));
		displayPathConstraints(mp);
	}

}

void displayPathConstraints(clopema_arm_navigation::ClopemaMotionPlan & mp) {
	if (mp.request.motion_plan_req.path_constraints.position_constraints.size() == 0)
		return;
	visualization_msgs::MarkerArray arr;
	visualization_msgs::Marker marker;
	marker.header.frame_id = mp.request.motion_plan_req.path_constraints.position_constraints[0].header.frame_id;
	marker.header.stamp = ros::Time::now();
	marker.ns = "basic_shapes";
	marker.id = 0;
	marker.type = visualization_msgs::Marker::SPHERE;
	marker.action = visualization_msgs::Marker::ADD;
	marker.pose.position.x = mp.request.motion_plan_req.path_constraints.position_constraints.at(0).position.x;
	marker.pose.position.y = mp.request.motion_plan_req.path_constraints.position_constraints.at(0).position.y;
	marker.pose.position.z = mp.request.motion_plan_req.path_constraints.position_constraints.at(0).position.z;
	marker.pose.orientation.x = mp.request.motion_plan_req.path_constraints.position_constraints.at(0).constraint_region_orientation.x;
	marker.pose.orientation.y = mp.request.motion_plan_req.path_constraints.position_constraints.at(0).constraint_region_orientation.y;
	marker.pose.orientation.z = mp.request.motion_plan_req.path_constraints.position_constraints.at(0).constraint_region_orientation.z;
	marker.pose.orientation.w = mp.request.motion_plan_req.path_constraints.position_constraints.at(0).constraint_region_orientation.w;
	marker.scale.x = 2 * mp.request.motion_plan_req.path_constraints.position_constraints.at(0).constraint_region_shape.dimensions.at(0);
	marker.scale.y = 2 * mp.request.motion_plan_req.path_constraints.position_constraints.at(0).constraint_region_shape.dimensions.at(1);
	marker.scale.z = 2 * mp.request.motion_plan_req.path_constraints.position_constraints.at(0).constraint_region_shape.dimensions.at(2);

	marker.color.r = 0.0f;
	marker.color.g = 1.0f;
	marker.color.b = 0.0f;
	marker.color.a = 0.5;

	marker.lifetime = ros::Duration();

	arr.markers.push_back(marker);
	marker_pub->publish(arr);

}
void setPathConstraints(clopema_arm_navigation::ClopemaMotionPlan & mp) {
	/*
	 mp.request.motion_plan_req.path_constraints.orientation_constraints.resize(1);
	 mp.request.motion_plan_req.path_constraints.orientation_constraints[0].header.frame_id = "r1_ee";
	 mp.request.motion_plan_req.path_constraints.orientation_constraints[0].header.stamp = ros::Time::now();
	 mp.request.motion_plan_req.path_constraints.orientation_constraints[0].link_name = "r2_ee";
	 mp.request.motion_plan_req.path_constraints.orientation_constraints[0].orientation.x = 0.0;
	 mp.request.motion_plan_req.path_constraints.orientation_constraints[0].orientation.y = 0.0;
	 mp.request.motion_plan_req.path_constraints.orientation_constraints[0].orientation.z = 0.0;
	 mp.request.motion_plan_req.path_constraints.orientation_constraints[0].orientation.w = 1.0;
	 mp.request.motion_plan_req.path_constraints.orientation_constraints[0].type = arm_navigation_msgs::OrientationConstraint::HEADER_FRAME;
	 mp.request.motion_plan_req.path_constraints.orientation_constraints[0].absolute_roll_tolerance = 1.4;
	 mp.request.motion_plan_req.path_constraints.orientation_constraints[0].absolute_pitch_tolerance = 1.4;
	 mp.request.motion_plan_req.path_constraints.orientation_constraints[0].absolute_yaw_tolerance = 1.4;*/

	mp.request.motion_plan_req.path_constraints.position_constraints.resize(1);
	mp.request.motion_plan_req.path_constraints.position_constraints[0].header.frame_id = "r1_ee";
	mp.request.motion_plan_req.path_constraints.position_constraints[0].header.stamp = ros::Time::now();
	mp.request.motion_plan_req.path_constraints.position_constraints[0].link_name = "r2_ee";
	mp.request.motion_plan_req.path_constraints.position_constraints[0].position.x = 0.0;
	mp.request.motion_plan_req.path_constraints.position_constraints[0].position.y = 0.0;
	mp.request.motion_plan_req.path_constraints.position_constraints[0].position.z = 0.0;
	mp.request.motion_plan_req.path_constraints.position_constraints[0].constraint_region_shape.type = arm_navigation_msgs::Shape::SPHERE;
	mp.request.motion_plan_req.path_constraints.position_constraints[0].constraint_region_shape.dimensions.push_back(0.41); //radius
	mp.request.motion_plan_req.path_constraints.position_constraints[0].constraint_region_shape.dimensions.push_back(0.41);
	mp.request.motion_plan_req.path_constraints.position_constraints[0].constraint_region_shape.dimensions.push_back(0.41);
	mp.request.motion_plan_req.path_constraints.position_constraints[0].constraint_region_orientation.x = 0.0;
	mp.request.motion_plan_req.path_constraints.position_constraints[0].constraint_region_orientation.y = 0.0;
	mp.request.motion_plan_req.path_constraints.position_constraints[0].constraint_region_orientation.z = 0.0;
	mp.request.motion_plan_req.path_constraints.position_constraints[0].constraint_region_orientation.w = 1.0;
	mp.request.motion_plan_req.path_constraints.position_constraints[0].weight = 1.0;

}

void doneCb(const actionlib::SimpleClientGoalState& state, const control_msgs::FollowJointTrajectoryResultConstPtr & result) {
	ROS_INFO("Finished in state [%s:%s]", state.toString().c_str(), state.text_.data());
	ROS_INFO("Result - error code: %d", result->error_code);
	cm->setServoPowerOff(false);
}

bool planToStart(clopema_arm_navigation::ClopemaMotionPlan & mp, ClopemaMove & cmove) {
	mp.request.motion_plan_req.group_name = "arms";
	mp.request.motion_plan_req.allowed_planning_time = ros::Duration(15.0);
	mp.request.motion_plan_req.num_planning_attempts = 5;
	mp.request.motion_plan_req.planner_id = "EST";
	if (!cmove.getRobotState(mp.request.motion_plan_req.start_state))
		return false;

	arm_navigation_msgs::SimplePoseConstraint desired_pose;
	desired_pose.header.frame_id = "base_link";
	desired_pose.header.stamp = ros::Time::now();
	desired_pose.link_name = "r2_ee";
	desired_pose.pose.position.x = 0.20;
	desired_pose.pose.position.y = -0.75;
	desired_pose.pose.position.z = 1.0;
	desired_pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(M_PI, 0, M_PI_2);
	desired_pose.absolute_position_tolerance.x = desired_pose.absolute_position_tolerance.y = desired_pose.absolute_position_tolerance.z = 0.02;
	desired_pose.absolute_roll_tolerance = desired_pose.absolute_pitch_tolerance = desired_pose.absolute_yaw_tolerance = 0.04;

	cmove.poseToClopemaMotionPlan(mp, desired_pose);
	desired_pose.link_name = "r1_ee";
	desired_pose.pose.position.x = -0.20;
	cmove.poseToClopemaMotionPlan(mp, desired_pose);

	ROS_INFO("Planning to the start position");
	if (!cmove.plan(mp))
		return false;
	trajectory_msgs::JointTrajectory wholeTraj = mp.response.joint_trajectory;
	mp.response.joint_trajectory = wholeTraj;
	return true;
}

bool planTrajectory(clopema_arm_navigation::ClopemaMotionPlan & mp, ClopemaMove & cmove) {
	mp.request.motion_plan_req.group_name = "arms";
	mp.request.motion_plan_req.planner_id = "EST";
	mp.request.motion_plan_req.num_planning_attempts = 5;
	mp.request.motion_plan_req.allowed_planning_time = ros::Duration(10.0);
	if (!cmove.getRobotState(mp.request.motion_plan_req.start_state))
		return false;

	arm_navigation_msgs::SimplePoseConstraint desired_pose;
	desired_pose.header.frame_id = "base_link";
	desired_pose.header.stamp = ros::Time::now();
	desired_pose.link_name = "r2_ee";
	desired_pose.pose.position.x = 0.615;
	desired_pose.pose.position.y = -0.55;
	desired_pose.pose.position.z = 1.0;
	desired_pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(M_PI, 0, -M_PI);
	desired_pose.absolute_position_tolerance.x = desired_pose.absolute_position_tolerance.y = desired_pose.absolute_position_tolerance.z = 0.02;
	desired_pose.absolute_roll_tolerance = desired_pose.absolute_pitch_tolerance = desired_pose.absolute_yaw_tolerance = 0.04;

	cmove.poseToClopemaMotionPlan(mp, desired_pose);
	desired_pose.link_name = "r1_ee";
	desired_pose.pose.position.y = -0.95;
	cmove.poseToClopemaMotionPlan(mp, desired_pose);

	ROS_INFO("Planning to the start position");
	if (!cmove.plan(mp))
		return false;
	trajectory_msgs::JointTrajectory wholeTraj = mp.response.joint_trajectory;
	mp.response.joint_trajectory = wholeTraj;
	return true;
}
