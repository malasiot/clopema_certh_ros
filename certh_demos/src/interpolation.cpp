#include <clopema_motoros/ClopemaMove.h>
#include <clopema_arm_navigation/ClopemaJointInterpolation.h>
#include <clopema_arm_navigation/ClopemaLinearInterpolation.h>

boost::shared_ptr<ClopemaMove> cm;

void doneCb(const actionlib::SimpleClientGoalState& state, const control_msgs::FollowJointTrajectoryResultConstPtr & result) {
	ROS_INFO("Finished in state [%s:%s]", state.toString().c_str(), state.text_.data());
	ROS_INFO("Result - error code: %d", result->error_code);
	cm->setServoPowerOff(false);
}

void printTraj(trajectory_msgs::JointTrajectory & traj) {
	ROS_INFO("----------Printing trajectory----------");
	for (unsigned int i = 0; i < traj.joint_names.size(); i++) {
		std::cout << traj.joint_names.at(i).data() << " ";
	}
	std::cout << std::endl;
	for (unsigned int i = 0; i < traj.points.size(); i++) {
		for (unsigned int j = 0; j < traj.joint_names.size(); j++) {
			std::cout << traj.points.at(i).positions.at(j) << " ";
		}
		std::cout << std::endl;
	}
}

bool moveTo(const geometry_msgs::Pose & pose, std::string base_link_name, std::string ik_link_name, std::string group) {
	clopema_arm_navigation::ClopemaMotionPlan mp;
	mp.request.motion_plan_req.group_name = group;
	mp.request.motion_plan_req.allowed_planning_time = ros::Duration(20.0);
	if (!cm->getRobotState(mp.request.motion_plan_req.start_state))
		return false;

	arm_navigation_msgs::SimplePoseConstraint desired_pose;
	desired_pose.header.frame_id = base_link_name;
	desired_pose.header.stamp = ros::Time::now();
	desired_pose.link_name = ik_link_name;
	desired_pose.pose = pose;
	desired_pose.absolute_position_tolerance.x = desired_pose.absolute_position_tolerance.y = desired_pose.absolute_position_tolerance.z = 0.02;
	desired_pose.absolute_roll_tolerance = desired_pose.absolute_pitch_tolerance = desired_pose.absolute_yaw_tolerance = 0.04;

	cm->poseToClopemaMotionPlan(mp, desired_pose);
	if (!cm->plan(mp))
		return false;
	trajectory_msgs::JointTrajectory traj = mp.response.joint_trajectory;

	control_msgs::FollowJointTrajectoryGoal goal;
	if (traj.points.size() > 2) {
		goal.trajectory = traj;
		cm->cmc_.sendGoal(goal, &doneCb);
		cm->cmc_.waitForResult(ros::Duration(60));
	}
	ros::Duration(1.0).sleep();

	return true;
}

void joint_interpolation() {
	sensor_msgs::JointState f, g;
	f.name.push_back("r2_joint_s");
	f.name.push_back("r2_joint_l");
	f.name.push_back("r2_joint_u");
	f.name.push_back("r2_joint_r");
	f.name.push_back("r2_joint_b");
	f.name.push_back("r2_joint_t");
	f.header.frame_id = "base_link";
	f.position.resize(f.name.size());
	f.velocity.resize(f.name.size());
	f.effort.resize(f.name.size());

	g.name = f.name;
	g.header = f.header;
	g.position.resize(g.name.size());
	g.velocity.resize(g.name.size());
	g.effort.resize(g.name.size());

	g.position[0] = -0.5;
	g.position[1] = 0.5;
	g.position[2] = 0.5;
	g.position[3] = 0.5;
	g.position[4] = 0.5;
	g.position[5] = 0.5;

	clopema_arm_navigation::ClopemaJointInterpolation srv;
	srv.request.poses.push_back(f);
	srv.request.poses.push_back(g);
	srv.request.poses.push_back(f);
	srv.request.poses.push_back(g);
	srv.request.poses.push_back(f);

	if (!ros::service::call("/clopema_planner/joint_interpolation", srv)) {
		ROS_ERROR("Can't call joint_interpolation service.");
		return;
	}

	if (!srv.response.error.empty())
		ROS_WARN_STREAM("error: "<<srv.response.error);

	control_msgs::FollowJointTrajectoryGoal goal;
	if (srv.response.joint_trajectory.points.size() > 2) {
		goal.trajectory = srv.response.joint_trajectory;
		ROS_INFO("Sending trajectory size: %d", (int)goal.trajectory.points.size());
		cm->cmc_.sendGoal(goal, &doneCb);
		cm->cmc_.waitForResult(ros::Duration(60));
	}

}

void linear_interpolation() {
	clopema_arm_navigation::ClopemaLinearInterpolation srv;
	geometry_msgs::Pose f, g;
	f.position.x = 0.5;
	f.position.y = -1.0;
	f.position.z = 1.0;
	f.orientation = tf::createQuaternionMsgFromRollPitchYaw(M_PI, 0, 0);

	g.position.x = 0.5;
	g.position.y = -1.0;
	g.orientation = tf::createQuaternionMsgFromRollPitchYaw(M_PI, 0, 0);
	g.position.z = 0.75;

	srv.request.poses.push_back(f);
	srv.request.poses.push_back(g);
	srv.request.poses.push_back(f);
	srv.request.poses.push_back(g);
	srv.request.poses.push_back(f);
	srv.request.ik_link_name = "r2_ee";
	srv.request.header.frame_id = "base_link";

	if (!ros::service::call("/clopema_planner/linear_interpolation", srv)) {
		ROS_ERROR("Can't call linear_interpolation service.");
		return;
	}

	if (!srv.response.error.empty())
		ROS_WARN_STREAM("error: "<<srv.response.error);

	if (!moveTo(srv.request.poses[0], srv.request.header.frame_id, srv.request.ik_link_name, "r2_arm")) {
		ROS_ERROR_STREAM("Can't plan to the start position");
		return;
	}


	control_msgs::FollowJointTrajectoryGoal goal;
	if (srv.response.joint_trajectory.points.size() > 2) {
		goal.trajectory = srv.response.joint_trajectory;
		ROS_INFO("Sending trajectory size: %d", (int)goal.trajectory.points.size());
		cm->cmc_.sendGoal(goal, &doneCb);
		cm->cmc_.waitForResult(ros::Duration(60));
	}
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "test_interpolation");
	ros::NodeHandle n;
	cm.reset(new ClopemaMove);

	linear_interpolation();
	//joint_interpolation();
}
