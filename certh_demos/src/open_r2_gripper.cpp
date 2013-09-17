#include <clopema_motoros/WriteIO.h>
#include <ros/ros.h>

using namespace std;

int main(int argc, char **argv) {

    ros::init(argc, argv, "open_r2_gripper");
    ros::NodeHandle nh;
    clopema_motoros::WriteIO openGripper;
    openGripper.request.address = 10026;
    for(unsigned int i = 0 ; i < 4 ; i++){
        openGripper.request.value = false;
        if (!ros::service::call("/write_io", openGripper)) {
            ROS_ERROR("Can't call service write_io");
            return -1;
        }
        ros::Duration(0.1).sleep();

        openGripper.request.value = true;
        ros::service::waitForService("/write_io");
        if (!ros::service::call("/write_io", openGripper)) {
            ROS_ERROR("Can't call service write_io");
            return -1;
        }
        ros::Duration(0.5).sleep();
    }
    ros::shutdown();
    return 1;

}

