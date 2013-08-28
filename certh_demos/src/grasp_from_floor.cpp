#include "robot_helpers/Unfold.h"

using namespace robot_helpers;
int main(int argc, char **argv) {

    ros::init(argc, argv, "unfolding");
    ros::NodeHandle nh;
    ros::Publisher marker_pub;
    marker_pub = nh.advertise<visualization_msgs::Marker>("/visualization_marker", 0);
    robot_helpers::Unfold uf("r2",marker_pub );
    geometry_msgs::Pose pose;
    pose.orientation = uf.rotationMatrix3ToQuaternion(uf.vertical());
    pose.position.x = 0;
    pose.position.y = -1;
    pose.position.z = 1.1;
    moveArm(pose, uf.getHoldingArm());



    setServoPowerOff(true);
}
