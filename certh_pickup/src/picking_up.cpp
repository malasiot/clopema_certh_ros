#include "ros/ros.h"
#include "PickUp.h"
#include <robot_helpers/Robot.h>

int main(int argc, char **argv) {


    ros::init(argc, argv, "pick_cloth_from_table");
    ros::NodeHandle nh;

    PickUp pick("r2");

   // robot_helpers::setRobotSpeed(1) ;

    pick.graspClothFromTable();
    setServoPowerOff();

    return 0 ;

}
