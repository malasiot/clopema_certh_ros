#ifndef __ROBOT_HELPERS_GEOMETRY_H__
#define __ROBOT_HELPERS_GEOMETRY_H__

#include <Eigen/Geometry>
#include <iostream>
#include <geometry_msgs/Pose.h>

namespace robot_helpers {

std::ostream &operator << (std::ostream &strm, const Eigen::Quaterniond &q) ;

// Conversion from RPY to Eigen quaternion

Eigen::Quaterniond quatFromRPY(double roll, double pitch, double yaw) ;
void rpyFromQuat(const Eigen::Quaterniond &q, double &roll, double &pitch, double &yaw) ;

// a rotation that will align the Z axis with the given direction

Eigen::Quaterniond lookAt(const Eigen::Vector3d &dir, double roll = 0.0) ;

geometry_msgs::Pose eigenPoseToROS(const Eigen::Vector3d &pos, const Eigen::Quaterniond &orient) ;

} // namespace robot_helpers

#endif
