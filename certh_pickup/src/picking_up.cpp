#include "ros/ros.h"
#include "PickUp.h"
#include <robot_helpers/Robot.h>
#include <camera_helpers/OpenNIServiceClient.h>

int main(int argc, char **argv) {


    ros::init(argc, argv, "pick_cloth_from_table");
    ros::NodeHandle nh;
    system("rosrun camera_helpers openni_service xtion2 &") ;
    ros::Duration(3).sleep() ;

    float tableHeight = 0.725 ;
    addBoxToCollisionModel(1.3, 0, tableHeight/2.0, 0.8, 0.8, tableHeight );
    setGripperState("r2", false) ;
    PickUp pick("r2");
    pick.graspClothFromTable();
    camera_helpers::openni::disconnect(pick.camera);
    //resetCollisionModel() ;
    //setServoPowerOff();

    return 0 ;

}
