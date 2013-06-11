#include <clopema_motoros/ClopemaMove.h>
#include <arm_navigation_msgs/GetPlanningScene.h>
#include <kdl/frames.hpp>
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_kdl.h>

boost::shared_ptr<ClopemaMove> cm;
boost::shared_ptr<ros::Publisher> marker_pub;
boost::shared_ptr<tf::TransformBroadcaster> tf_brod;

KDL::Frame pose;
double grasp_offset_minus = 0.2; //start of the linear trajectory
double grasp_offset_plus = 0.2; //depth

bool planTrajectory(clopema_arm_navigation::ClopemaMotionPlan & mp, ClopemaMove & cmove);
void doneCb(const actionlib::SimpleClientGoalState& state, const control_msgs::FollowJointTrajectoryResultConstPtr & result);

void setPathConstraints(clopema_arm_navigation::ClopemaMotionPlan & mp);
bool planToStart(clopema_arm_navigation::ClopemaMotionPlan & mp, ClopemaMove & cmove);
void displayPathConstraints(clopema_arm_navigation::ClopemaMotionPlan & mp);

int main(int argc, char **argv) {
	ros::init(argc, argv, "pick_up");
	ros::NodeHandle n;
	marker_pub.reset(new ros::Publisher);
	*marker_pub = n.advertise<visualization_msgs::MarkerArray>("state_validity_markers_array", 128);
	cm.reset(new ClopemaMove);
	clopema_arm_navigation::ClopemaMotionPlan mp;
	tf_brod.reset(new tf::TransformBroadcaster);
	ros::Duration(1.0).sleep();

	/*-----------------------
	 * Setting pose which should by reached (z-axis is pointing the direction of the grasping)
	 */
	pose.Identity();
	pose.p = KDL::Vector(0.5, -1.45, 1.0);
	pose.M = KDL::Rotation::RPY(M_PI / 2, 0, 0);

	tf::Transform tf;
	tf::TransformKDLToTF(pose, tf);
	tf_brod->sendTransform(tf::StampedTransform(tf, ros::Time::now(), "base_link", "req_pose"));
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
		cm->cmc_.waitForResult(ros::Duration(60));
	}
	mp.request.motion_plan_req.goal_constraints.joint_constraints.clear();
	mp.request.motion_plan_req.goal_constraints.position_constraints.clear();
	mp.request.motion_plan_req.goal_constraints.orientation_constraints.clear();

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
	marker.type = visualization_msgs::Marker::CUBE;
	marker.action = visualization_msgs::Marker::ADD;
	marker.pose.position.x = mp.request.motion_plan_req.path_constraints.position_constraints.at(0).position.x;
	marker.pose.position.y = mp.request.motion_plan_req.path_constraints.position_constraints.at(0).position.y;
	marker.pose.position.z = mp.request.motion_plan_req.path_constraints.position_constraints.at(0).position.z;
	marker.pose.orientation.x = mp.request.motion_plan_req.path_constraints.position_constraints.at(0).constraint_region_orientation.x;
	marker.pose.orientation.y = mp.request.motion_plan_req.path_constraints.position_constraints.at(0).constraint_region_orientation.y;
	marker.pose.orientation.z = mp.request.motion_plan_req.path_constraints.position_constraints.at(0).constraint_region_orientation.z;
	marker.pose.orientation.w = mp.request.motion_plan_req.path_constraints.position_constraints.at(0).constraint_region_orientation.w;
	marker.scale.x = mp.request.motion_plan_req.path_constraints.position_constraints.at(0).constraint_region_shape.dimensions.at(0);
	marker.scale.y = mp.request.motion_plan_req.path_constraints.position_constraints.at(0).constraint_region_shape.dimensions.at(1);
	marker.scale.z = mp.request.motion_plan_req.path_constraints.position_constraints.at(0).constraint_region_shape.dimensions.at(2);

	marker.color.r = 0.0f;
	marker.color.g = 1.0f;
	marker.color.b = 0.0f;
	marker.color.a = 0.5;

	marker.lifetime = ros::Duration();

	arr.markers.push_back(marker);
	marker_pub->publish(arr);

}
void setPathConstraints(clopema_arm_navigation::ClopemaMotionPlan & mp) {

	KDL::Frame moveX = KDL::Frame::Identity();
	moveX.p.z((grasp_offset_plus - grasp_offset_minus) / 2);
	KDL::Frame f = pose * moveX; //the result pose
	geometry_msgs::Quaternion q;
	f.M.GetQuaternion(q.x, q.y, q.z, q.w);
	/*desired_pose.pose.position.x = f.p.x();
	 desired_pose.pose.position.y = f.p.y();
	 desired_pose.pose.position.z = f.p.z();
	 f.M.GetQuaternion(desired_pose.pose.orientation.x, desired_pose.pose.orientation.y, desired_pose.pose.orientation.z, desired_pose.pose.orientation.w);

	mp.request.motion_plan_req.path_constraints.orientation_constraints.resize(1);
	mp.request.motion_plan_req.path_constraints.orientation_constraints[0].header.frame_id = "base_link";
	mp.request.motion_plan_req.path_constraints.orientation_constraints[0].header.stamp = ros::Time::now();
	mp.request.motion_plan_req.path_constraints.orientation_constraints[0].link_name = "r2_ee";
	mp.request.motion_plan_req.path_constraints.orientation_constraints[0].orientation = q;
	mp.request.motion_plan_req.path_constraints.orientation_constraints[0].type = arm_navigation_msgs::OrientationConstraint::HEADER_FRAME;
	mp.request.motion_plan_req.path_constraints.orientation_constraints[0].absolute_roll_tolerance = 0.4; //~23 deg
	mp.request.motion_plan_req.path_constraints.orientation_constraints[0].absolute_pitch_tolerance = 0.4;
	mp.request.motion_plan_req.path_constraints.orientation_constraints[0].absolute_yaw_tolerance = 0.4;
 */
	mp.request.motion_plan_req.path_constraints.position_constraints.resize(1);
	mp.request.motion_plan_req.path_constraints.position_constraints[0].header.frame_id = "base_link";
	mp.request.motion_plan_req.path_constraints.position_constraints[0].header.stamp = ros::Time::now();
	mp.request.motion_plan_req.path_constraints.position_constraints[0].link_name = "r2_ee";
	mp.request.motion_plan_req.path_constraints.position_constraints[0].position.x = f.p.x();
	mp.request.motion_plan_req.path_constraints.position_constraints[0].position.y = f.p.y();
	mp.request.motion_plan_req.path_constraints.position_constraints[0].position.z = f.p.z();
	mp.request.motion_plan_req.path_constraints.position_constraints[0].constraint_region_shape.type = arm_navigation_msgs::Shape::BOX;
	mp.request.motion_plan_req.path_constraints.position_constraints[0].constraint_region_shape.dimensions.push_back(0.1);
	mp.request.motion_plan_req.path_constraints.position_constraints[0].constraint_region_shape.dimensions.push_back(0.1);
	mp.request.motion_plan_req.path_constraints.position_constraints[0].constraint_region_shape.dimensions.push_back((grasp_offset_plus + grasp_offset_minus)*2);
	mp.request.motion_plan_req.path_constraints.position_constraints[0].constraint_region_orientation = q;
	mp.request.motion_plan_req.path_constraints.position_constraints[0].weight = 1.0;

}

void doneCb(const actionlib::SimpleClientGoalState& state, const control_msgs::FollowJointTrajectoryResultConstPtr & result) {
	ROS_INFO("Finished in state [%s:%s]", state.toString().c_str(), state.text_.data());
	ROS_INFO("Result - error code: %d", result->error_code);
	cm->setServoPowerOff(false);
}

bool planToStart(clopema_arm_navigation::ClopemaMotionPlan & mp, ClopemaMove & cmove) {
	mp.request.motion_plan_req.group_name = "r2_arm";
	mp.request.motion_plan_req.allowed_planning_time = ros::Duration(15.0);
	if (!cmove.getRobotState(mp.request.motion_plan_req.start_state))
		return false;

	KDL::Frame moveX = KDL::Frame::Identity();
	moveX.p.z(-grasp_offset_minus);
	KDL::Frame f = pose * moveX; //the result pose

	tf::Transform tf;
	tf::TransformKDLToTF(f, tf);
	tf_brod->sendTransform(tf::StampedTransform(tf, ros::Time::now(), "base_link", "req_pose_off_min"));

	arm_navigation_msgs::SimplePoseConstraint desired_pose;
	desired_pose.header.frame_id = "base_link";
	desired_pose.header.stamp = ros::Time::now();
	desired_pose.link_name = "r2_ee";
	desired_pose.pose.position.x = f.p.x();
	desired_pose.pose.position.y = f.p.y();
	desired_pose.pose.position.z = f.p.z();
	f.M.GetQuaternion(desired_pose.pose.orientation.x, desired_pose.pose.orientation.y, desired_pose.pose.orientation.z, desired_pose.pose.orientation.w);
	desired_pose.absolute_position_tolerance.x = desired_pose.absolute_position_tolerance.y = desired_pose.absolute_position_tolerance.z = 0.02;
	desired_pose.absolute_roll_tolerance = desired_pose.absolute_pitch_tolerance = desired_pose.absolute_yaw_tolerance = 0.04;

	cmove.poseToClopemaMotionPlan(mp, desired_pose);

	ROS_INFO("Planning to the start position");
	if (!cmove.plan(mp))
		return false;
	trajectory_msgs::JointTrajectory wholeTraj = mp.response.joint_trajectory;
	mp.response.joint_trajectory = wholeTraj;
	return true;
}

bool planTrajectory(clopema_arm_navigation::ClopemaMotionPlan & mp, ClopemaMove & cmove) {
	mp.request.motion_plan_req.group_name = "r2_arm";
	mp.request.motion_plan_req.allowed_planning_time = ros::Duration(60.0);
	if (!cmove.getRobotState(mp.request.motion_plan_req.start_state))
		return false;

	KDL::Frame moveX = KDL::Frame::Identity();
	moveX.p.z(grasp_offset_plus);
	KDL::Frame f = pose * moveX; //the result pose

	tf::Transform tf;
	tf::TransformKDLToTF(f, tf);
	tf_brod->sendTransform(tf::StampedTransform(tf, ros::Time::now(), "base_link", "req_pose_off_min"));

	arm_navigation_msgs::SimplePoseConstraint desired_pose;
	desired_pose.header.frame_id = "base_link";
	desired_pose.header.stamp = ros::Time::now();
	desired_pose.link_name = "r2_ee";
	desired_pose.pose.position.x = f.p.x();
	desired_pose.pose.position.y = f.p.y();
	desired_pose.pose.position.z = f.p.z();
	f.M.GetQuaternion(desired_pose.pose.orientation.x, desired_pose.pose.orientation.y, desired_pose.pose.orientation.z, desired_pose.pose.orientation.w);
	desired_pose.absolute_position_tolerance.x = desired_pose.absolute_position_tolerance.y = desired_pose.absolute_position_tolerance.z = 0.02;
	desired_pose.absolute_roll_tolerance = desired_pose.absolute_pitch_tolerance = desired_pose.absolute_yaw_tolerance = 0.04;

	cmove.poseToClopemaMotionPlan(mp, desired_pose);

	ROS_INFO("Planning to the end position");
	if (!cmove.plan(mp))
		return false;
	trajectory_msgs::JointTrajectory wholeTraj = mp.response.joint_trajectory;
	mp.response.joint_trajectory = wholeTraj;
	return true;
}
