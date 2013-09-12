#include "ros/ros.h"
#include "PickUp.h"

int main(int argc, char **argv) {


    ros::init(argc, argv, "pick_cloth_from_table");
    ros::NodeHandle nh;

    PickUp pick("r2");
    pick.graspClothFromTable();
    setServoPowerOff();

    return 0 ;

}
