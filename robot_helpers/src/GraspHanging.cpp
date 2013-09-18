#include "robot_helpers/Geometry.h"
#include "robot_helpers/Planner.h"
#include "robot_helpers/Planners.h"
#include <vector>

#include <boost/random/mersenne_twister.hpp>
#include <boost/random/uniform_real.hpp>

using namespace std ;
using namespace Eigen ;

namespace robot_helpers {


class GraspHangingGoalRegion: public GoalRegion {
public:
    GraspHangingGoalRegion(GraspHangingPlanner *planner_, const Affine3d &orig_, const Vector3d &p_, const Vector3d &dir_):
        planner(planner_), orig(orig_), p(p_), dir(dir_), gen(time(0))
    {

    }

    void sample(std::vector<double> &xyz_rpy) ;


    const Affine3d &orig ;
    Vector3d p, dir ;

    GraspHangingPlanner *planner ;
    boost::mt19937 gen;

};




void GraspHangingGoalRegion::sample(std::vector<double> &xyz_rpy)
{
    // compute the approach pose so that the gripper is perpendicular to the given direction with a random slant with
    // respect to the horizontal plane

    Vector3d nz = -dir, na, nb ;
    nz.normalize() ;

    double q = sqrt(nz.x() * nz.x() + nz.y() * nz.y()) ;

    if ( q < 1.0e-4 )
    {
        na = Vector3d(0, 1, 0) ;
        nb = nz.cross(na) ;
    }
    else {

        na = Vector3d(-nz.y()/q, nz.x()/q, 0) ;
        nb = Vector3d(-nz.x() * nz.z()/q, -nz.y() * nz.z()/q, q) ;
    }

    Matrix3d r ;
    r << nz, nb, -na ;

    double angle = boost::uniform_real<double>(planner->pitch_tol_min, planner->pitch_tol_max)(gen) ;
    double pa = boost::uniform_real<double>(planner->yaw_tol_min, planner->yaw_tol_max)(gen) ;
    double roll = boost::uniform_real<double>(planner->roll_tol_min, planner->roll_tol_max)(gen) ;

    r =  Quaterniond(r) * /*AngleAxisd(angle, Eigen::Vector3d::UnitX()) * */ AngleAxisd(-roll, Eigen::Vector3d::UnitY()) ; /** AngleAxisd(pa, Eigen::Vector3d::UnitZ()) */

    // we allow the holding arm to move within a box with dimensions t x t x t around the current position

    boost::uniform_real<double> dist(-0.5, 0.5) ;
    Vector3d pos1_offset = Vector3d(planner->x_tol * dist(gen), planner->y_tol * dist(gen), planner->z_tol * dist(gen)) ;

    //Vector3d pos1_offset(0, 0, 0) ;

    // compute the pose of the holding arm (arm 1)

    Vector3d pos1 = pos1_offset + orig.translation() ;

    double roll1, pitch1, yaw1, x1, y1, z1 ;

    x1 = pos1.x() ; y1 = pos1.y() ; z1 = pos1.z() ;
    rpyFromQuat(Quaterniond(orig.rotation()) * AngleAxisd(roll, Eigen::Vector3d::UnitZ()), roll1, pitch1, yaw1) ;

    // compute the pose of the approach arm (arm 2)

    Vector3d offset = - r * Vector3d(0, 0, 1) * planner->offset ; // offset with respect to the target point

    Vector3d pos2 = offset + p + pos1_offset ;

    Matrix3d ri = r.inverse() ;

    Quaterniond q2(r) ;
    q2.normalize() ;

    double roll2, pitch2, yaw2, x2, y2, z2 ;

    x2 = pos2.x() ; y2 = pos2.y() ; z2 = pos2.z() ;
    rpyFromQuat(q2, roll2, pitch2, yaw2) ;

    if ( planner->arm == "r1" )
    {
        xyz_rpy.push_back(x1) ;  xyz_rpy.push_back(y1) ;   xyz_rpy.push_back(z1) ;
        xyz_rpy.push_back(roll1) ;  xyz_rpy.push_back(pitch1) ; xyz_rpy.push_back(yaw1) ;

        xyz_rpy.push_back(x2) ;  xyz_rpy.push_back(y2) ;   xyz_rpy.push_back(z2) ;
        xyz_rpy.push_back(roll2) ;  xyz_rpy.push_back(pitch2) ; xyz_rpy.push_back(yaw2) ;


    }
    else
    {
        xyz_rpy.push_back(x2) ;  xyz_rpy.push_back(y2) ;   xyz_rpy.push_back(z2) ;
        xyz_rpy.push_back(roll2) ;  xyz_rpy.push_back(pitch2) ; xyz_rpy.push_back(yaw2) ;

        xyz_rpy.push_back(x1) ;  xyz_rpy.push_back(y1) ;   xyz_rpy.push_back(z1) ;
        xyz_rpy.push_back(roll1) ;  xyz_rpy.push_back(pitch1) ; xyz_rpy.push_back(yaw1) ;

    }



}


GraspHangingPlanner::GraspHangingPlanner(KinematicsModel &model, const string &armName): kmodel(model), arm(armName) {

    IKSolverPtr solver_r1(new MA1400_R1_IKSolver) ;
    solver_r1->setKinematicModel(&kmodel);

    IKSolverPtr solver_r2(new MA1400_R2_IKSolver) ;
    solver_r2->setKinematicModel(&kmodel);

    pCtx.reset(new PlanningContextDual("arms", &kmodel, solver_r1, "r1_ee", solver_r2, "r2_ee" ) ) ;

    x_tol = y_tol = z_tol = 0.5 ;
    roll_tol_min = -M_PI/6.0 ;
    roll_tol_max =  M_PI/6.0 ;
    yaw_tol_min = -M_PI/30.0 ;
    yaw_tol_max =  M_PI/30.0 ;
    pitch_tol_min = -M_PI/4 ;
    pitch_tol_max = 0 ;
    offset = 0.05 ;

}



bool GraspHangingPlanner::plan(const Vector3d &p, const Vector3d &perp_dir, trajectory_msgs::JointTrajectory &traj)
{

    JointSpacePlanner planner(pCtx) ;

    // get pose of the end effector for the arm tip holding the cloth

    Affine3d orig_pose = kmodel.getWorldTransform(arm + "_ee") ;

    Vector3d rp = orig_pose.inverse() * p ;
    Vector3d rdir = orig_pose.rotation().inverse() * perp_dir ;

    GraspHangingGoalRegion rg(this, orig_pose, p, perp_dir) ;

    JointTrajectory traj_ ;

    bool rplan = planner.solve(rg, traj_) ;

    if ( rplan )
    {
 //   traj.completeTrajectory(kmodel.getJointState()) ;

        JointState rs = kmodel.getJointState() ;

        trajectory_msgs::JointTrajectory msg = traj_.toMsg(10) ;

        filterTrajectory("arms", msg, traj) ;
        JointState js(traj_.names, traj_.positions.back()) ;

        kmodel.setJointState(js) ;

        Affine3d final_pose = kmodel.getWorldTransform(arm + "_ee") ;

        kmodel.setJointState(rs);

        fp = final_pose * rp ;
        fdir = final_pose.rotation() * rdir ;
    }

    return rplan ;

}


}
