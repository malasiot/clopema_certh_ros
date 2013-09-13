#ifndef __ROBOT_HELPERS_PLANNERS_H__
#define __ROBOT_HELPERS_PLANNERS_H__

#include <robot_helpers/KinematicsInterface.h>
#include <robot_helpers/Planner.h>

namespace robot_helpers {


class GraspHangingPlanner {

public:
    GraspHangingPlanner( KinematicsModel &kmodel, const std::string &armName ) ;

    // Dual arm motion planar for grasping a point on a hanging piece of cloth at a position (p) and direction perpendicular to the given normal vector (perp_dir).
    // arm 1 (armName) is the arm grasping the cloth and arm 2 the arm that is doing the grasp. The planner will try to move arm1 inside a box of given dimensions and
    // also rotate arm1 tip link so that the requested point comes to a position where it is graspable by arm2.

    bool plan(const Eigen::Vector3d &p, const Eigen::Vector3d &perp_dir, trajectory_msgs::JointTrajectory &traj) ;

public:

    double x_tol, y_tol, z_tol ; // tolerance of the movement of arm1 around initial position
    double roll_tol_min, roll_tol_max, yaw_tol_min, yaw_tol_max, pitch_tol_min, pitch_tol_max ; // rotation tolerance of the approaching arm
    double offset ; // offset with respect to the target point


    // if plan succesfull these will contain the position and orientation of the target point

    Eigen::Vector3d fp ;
    Eigen::Vector3d fdir ;

private:
    friend class GraspHangingGoalRegion ;

    boost::shared_ptr<PlanningContext> pCtx ;
    KinematicsModel &kmodel ;
    std::string arm ;
} ;


}

#endif
