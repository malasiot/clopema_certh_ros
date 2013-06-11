#include <ros/ros.h>
#include <clopema_motoros/WriteIO.h>
#include <clopema_motoros/SetGripperState.h>


int main(int argc, char **argv) {
	ros::init(argc, argv, "move_pose_dual");
	ros::NodeHandle nh;
	

	while(1){
		std::cin.ignore();
		ros::service::waitForService("/r1_gripper/set_open");
		clopema_motoros::SetGripperState sopen;
		sopen.request.open=true;
		ros::service::call("/r1_gripper/set_open", sopen);
		ROS_INFO ("--->PLEASE HIT ENTER TO CONTINUE<---");
		std::cin.ignore();
		sopen.request.open=false;
		ros::service::call("/r1_gripper/set_open", sopen);
	}
}
