#ifndef __ROBOT_HELPERS_UTILS_H__
#define __ROBOT_HELPERS_UTILS_H__

#include "robot_helpers/Robot.h"

namespace robot_helpers {

// move robot to home position

bool moveHome(MoveRobot &cmove) ;

// move robot arm so that the gripper is pointing down and the tip is at the specified location

bool moveGripperPointingDown(MoveRobot &cmove, const std::string &armName, double X, double Y, double Z) ;


// rotate gripper the given amount

bool rotateGripper(MoveRobot &cmove, const std::string &armName, double theta) ;



}

#endif
