#include "ros/ros.h"
#include "PickUp.h"
#include <robot_helpers/Robot.h>

int main(int argc, char **argv) {


    ros::init(argc, argv, "pick_cloth_from_table");
    ros::NodeHandle nh;
    float tableHeight = 0.725 ;
    addBoxToCollisionModel(1.3, 0, tableHeight/2.0, 0.8, 0.8, tableHeight );
    PickUp pick("r2");

   // robot_helpers::setRobotSpeed(1) ;

    pick.graspClothFromTable();
    pick.grabber->disconnect();
    cout<< "grabber disconnected " << endl;
    resetCollisionModel() ;
    setServoPowerOff();

    return 0 ;

}
