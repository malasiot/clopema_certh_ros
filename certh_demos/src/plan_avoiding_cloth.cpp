#include "ros/ros.h"


#include <robot_helpers/Robot.h>
#include <robot_helpers/Utils.h>

using namespace robot_helpers ;

bool moveArmHoriz(MoveRobot &cmove, const std::string &armName, double X, double Y, double Z)
{


    Eigen::Quaterniond q ;

    if ( armName == "r2" )
        q = Eigen::AngleAxisd(-M_PI/2, Eigen::Vector3d::UnitY());
    else
        q = Eigen::AngleAxisd(M_PI/2, Eigen::Vector3d::UnitY());

    return moveGripper(cmove, armName, Eigen::Vector3d(X, Y, Z), q) ;
}



int main(int argc, char *argv[])
{
    ros::init(argc, argv, "qt_ros_test") ;
    ros::NodeHandle nh ;

    MoveRobot mv ;

    moveArmHoriz(mv, "r2", -0.5, -0.4, 0.9) ;

    addConeToCollisionModel("r1", 0.6, 0.2) ;

    moveArmHoriz(mv, "r2", -0.5, -1.25, 0.9) ;

    resetCollisionModel() ;

    moveArmHoriz(mv, "r2", -0.5, -0.6, 0.9) ;




}
