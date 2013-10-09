#include <ros/ros.h>

#include <robot_helpers/Robot.h>
#include <robot_helpers/Utils.h>
#include <robot_helpers/Geometry.h>
using namespace std;
using namespace robot_helpers;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "approach_Table");

  float tableHeight = 0.685 ; //0.695
  resetCollisionModel() ;
  addBoxToCollisionModel(0, -1.1, tableHeight/2.0, 0.8, 0.8, tableHeight);
  attachBoxToXtionInCollisionModel("r2");
  MoveRobot cmove;
  moveHome(cmove);
  setServoPowerOff();
 // resetCollisionModel() ;
  return 0;


}

