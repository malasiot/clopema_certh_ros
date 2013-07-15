#ifndef __ROBOT_HELPERS_UTILS_H__
#define __ROBOT_HELPERS_UTILS_H__

#include "robot_helpers/Robot.h"
#include <Eigen/Geometry>

namespace robot_helpers {

// planned end-point to reach given pose

bool planArmToPose(const std::string &armName, const Eigen::Vector3d &pos, const Eigen::Quaterniond &q, trajectory_msgs::JointTrajectory &traj) ;

// move robot to home position

bool moveHome(MoveRobot &cmove) ;

// move robot arm so that the tip of the gripper is at pos with orientation q

bool moveGripper(MoveRobot &cmove, const std::string &armName, const Eigen::Vector3d &pos, const Eigen::Quaterniond &q) ;

// move robot arm so that the gripper is pointing down and the tip is at the specified location

bool moveGripperPointingDown(MoveRobot &cmove, const std::string &armName, double X, double Y, double Z) ;

// rotate gripper the given amount

bool rotateGripper(MoveRobot &cmove, const std::string &armName, double theta) ;

// We add a collision object as attached object to the gripper. the parent frame is set to the baselink since otherwise the
// cone will rotate as the gripper moves. Thus this function may be used for one arm static or by calling resetCollisionModel, moving the arm,
// and then setting again the collision object. This is a peculiarity of non-rigid attachement.

bool addConeToCollisionModel(const std::string &armName, double length, double radius) ;
bool addSphereToCollisionModel(const std::string &armName, double radius);

bool resetCollisionModel() ;

// get inverse kinematics silution for the tip of given arm

bool getIK(const std::string &armName, const Eigen::Vector3d pos, const Eigen::Quaterniond &q, sensor_msgs::JointState &state) ;

}

#endif
