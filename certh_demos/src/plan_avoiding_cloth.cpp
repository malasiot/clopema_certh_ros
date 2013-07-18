#include "ros/ros.h"


#include <robot_helpers/Robot.h>
#include <robot_helpers/Utils.h>
#include <robot_helpers/Geometry.h>


using namespace robot_helpers ;
using namespace std;
using namespace Eigen;

bool moveArmHoriz(MoveRobot &cmove, const std::string &armName, double X, double Y, double Z, double theta = 0.0)
{

    Eigen::Quaterniond q ;

    if ( armName == "r2" )
        q = lookAt(Vector3d(-1, 0, 0)) ;
    else
        q = lookAt(Vector3d(1, 0, 0)) ;

    return moveGripper(cmove, armName, Eigen::Vector3d(X, Y, Z), q) ;
}


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "plan_avoiding_cloth") ;
    ros::NodeHandle nh ;

    MoveRobot mv ;

    moveHome(mv) ;

    moveArmHoriz(mv, "r2", -0.5, -0.4, 0.9) ;

    addConeToCollisionModel("r1", 0.6, 0.2) ;

    moveArmHoriz(mv, "r2", -0.5, -1.25, 0.9) ;

    resetCollisionModel() ;

    moveArmHoriz(mv, "r2", -0.5, -0.6, 0.9) ;
}


