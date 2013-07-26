#ifndef __ROBOT_HELPERS_JOINT_TRAJECTORY_H__
#define __ROBOT_HELPERS_JOINT_TRAJECTORY_H__

#include <trajectory_msgs/JointTrajectory.h>
#include <robot_helpers/KinematicsInterface.h>

namespace robot_helpers {

class JointTrajectory {
public:

    JointTrajectory() {}

    JointTrajectory(const std::vector<std::string> &joint_names) {}

    JointTrajectory(const trajectory_msgs::JointTrajectory &ros_traj) ;

    int size() const {
        return positions.size() ;
    }

    trajectory_msgs::JointTrajectory toMsg(double time_scale = 1.0) const ;

    void addPoint(double t, const JointState &state) ;

    void completeTrajectory(const JointState &rs) ;

    // linear interpolation of joint at time t [0, 1]
    double lerp(const std::string &joint, double t) const ;

    std::vector<std::string> names ;
    std::vector< std::vector<double> > positions ;
    std::vector<double> times ;
};


















}

#endif
