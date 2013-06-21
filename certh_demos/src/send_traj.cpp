#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <arm_navigation_msgs/GetRobotState.h>
#include <clopema_planning_actions/GetRobotState.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <clopema_motoros/SetPowerOff.h>
#include <robot_helpers/Utils.h>

using namespace std;

int main(int argc, char **argv) {

    if (argc < 2) {
        ROS_ERROR("Not enough arguments: send_traj <R1rAxisDesiredPosition>");
        return -1;
    }

    ros::init(argc, argv, "send_traj");
    ros::NodeHandle nh;
    ROS_INFO("Send_traj node started");

    cout << robot_helpers::getCurrentPose("r1").translation() << endl ;
    //Create client
    actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> move("/clopema_controller/follow_joint_trajectory", true);
    move.waitForServer();
    ROS_INFO("Connected to server");

    //Get current robot position
    ros::service::waitForService("/environment_server/get_robot_state");
    arm_navigation_msgs::GetRobotState r;
    if (!ros::service::call("/environment_server/get_robot_state", r)) {
        ROS_ERROR("Can't get current robot state.");
        return -1;
    }
    //~ ------------------------------
  //Get current robot position VIA CLOPEMA
    ros::service::waitForService("/clopema_planning_actions/get_robot_state");
    clopema_planning_actions::GetRobotState CRS;
    if (!ros::service::call("/clopema_planning_actions/get_robot_state", CRS)) {
        ROS_ERROR("Can't get current robot state.");
        return -1;
    }
	cout<< " ---- > " << CRS.response.robot_2_pose.pose.position.x<<"\n";
	cout<< " ---- > " << CRS.response.robot_2_pose.pose.position.y<<"\n";
	cout<< " ---- > " << CRS.response.robot_2_pose.pose.position.z<<"\n";
    
    //Get actual value of joint r1_joint_r (first robot R-axis)
    vector<string> names = r.response.robot_state.joint_state.name;
    int pos = -1;
    for (unsigned int i = 0; i < names.size(); i++)
        if (names.at(i) == "floating_trans_x")
            pos = i;



    if (pos == -1) {
        ROS_ERROR("Robot state reading error.");
        return -1;
    }
    double rAxisActual = r.response.robot_state.joint_state.position.at(pos);

    //Plan trajectory
    trajectory_msgs::JointTrajectory traj;
    traj.joint_names.push_back("r1_joint_t");

    std::stringstream rAxisDesired(argv[1]);
    double rAxisDiff = atof(rAxisDesired.str().data()) - rAxisActual;
    int samples = 10;
    samples--;
    for (int i = 0; i <= samples; i++) {
        trajectory_msgs::JointTrajectoryPoint p;
        p.positions.push_back(rAxisActual + rAxisDiff / samples * i);
        p.velocities.push_back(0.0);
        p.accelerations.push_back(0.0);
        traj.points.push_back(p);
    }
    traj.header.stamp = ros::Time::now();

    //Create goal and send it to the controller
    control_msgs::FollowJointTrajectoryGoal goal;
    goal.trajectory = traj;
    move.sendGoal(goal);

    bool finished_within_time = move.waitForResult(ros::Duration(45.0));
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
     
    
    //Set servo power off
	clopema_motoros::SetPowerOff soff;
	soff.request.force = false;
	ros::service::waitForService("/joint_trajectory_action/set_power_off");
	if (!ros::service::call("/joint_trajectory_action/set_power_off", soff)) {
		ROS_ERROR("Can't call service set_power_off");
		return -1;
	}
	return 1;
	
	ros::shutdown();
}

