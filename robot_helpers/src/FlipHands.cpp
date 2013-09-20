#include <robot_helpers/Planners.h>

#include <boost/random/mersenne_twister.hpp>
#include <boost/random/uniform_real.hpp>


using namespace Eigen ;
using namespace std ;

namespace robot_helpers {



class FlipHandsGoalRegion: public GoalRegion {
public:
    FlipHandsGoalRegion(FlipHandsPlanner *planner_, const Affine3d &pose1_, const Affine3d &pose2_):
        planner(planner_), p1(pose1_), p2(pose2_), gen(time(0))
    {

    }

    void sample(std::vector<double> &xyz_rpy) ;


    Affine3d p1, p2 ;

    FlipHandsPlanner *planner ;
    boost::mt19937 gen;

};




void FlipHandsGoalRegion::sample(std::vector<double> &xyz_rpy)
{
    // compute the approach pose so that the gripper is perpendicular to the given direction with a random slant with
    // respect to the horizontal plane

    boost::uniform_real<double> dist(-0.5, 0.5) ;
    Vector3d pos2_offset = Vector3d(planner->x_tol * dist(gen), planner->y_tol * dist(gen), planner->z_tol * dist(gen)) ;

    // compute the pose of the grasping arm (arm 2)

    Vector3d pos2 = pos2_offset + p1.translation() ;

    double d = (p1.translation() - p2.translation()).norm() ;

    Vector3d pos1 = p2.translation() + pos2_offset +
            Vector3d(boost::uniform_real<double>(-0.05, 0.05)(gen), boost::uniform_real<double>(-0.05, 0.05)(gen), boost::uniform_real<double>(0, 0.2)(gen));

    double roll1, pitch1, yaw1, x1, y1, z1 ;

    x1 = pos1.x(), y1 = pos1.y(), z1 = pos1.z() ;
    if ( planner->arm == "r1" )
        rpyFromQuat(lookAt(Vector3d(1, 0, 0)), roll1, pitch1, yaw1) ;
    else
        rpyFromQuat(lookAt(Vector3d(-1, 0, 0)), roll1, pitch1, yaw1) ;


    double roll2, pitch2, yaw2, x2, y2, z2 ;

    x2 = pos2.x() ; y2 = pos2.y() ; z2 = pos2.z() ;
    rpyFromQuat(Quaterniond(p1.rotation()), roll2, pitch2, yaw2) ;

    double pitch = boost::uniform_real<double>(planner->pitch_tol_min, planner->pitch_tol_max)(gen) ;
    double yaw = boost::uniform_real<double>(planner->yaw_tol_min, planner->yaw_tol_max)(gen) ;
    double roll = boost::uniform_real<double>(planner->roll_tol_min, planner->roll_tol_max)(gen) ;

    roll1 += roll ;  yaw1 += yaw ; pitch1 += pitch ;

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


FlipHandsPlanner::FlipHandsPlanner(KinematicsModel &model, const string &armName): kmodel(model), arm(armName) {


    IKSolverPtr solver_r1(new MA1400_R1_IKSolver) ;
     solver_r1->setKinematicModel(&kmodel);

     IKSolverPtr solver_r2(new MA1400_R2_IKSolver) ;
     solver_r2->setKinematicModel(&kmodel);

     pCtx.reset(new PlanningContextDual("arms", &kmodel, solver_r1, "r1_ee", solver_r2, "r2_ee")) ;

     min_dist_perc = 0.5 ;

     x_tol = y_tol = z_tol = 0.5 ;


     roll_tol_min = -M_PI/6.0 ;
     roll_tol_max =  M_PI/6.0 ;
     yaw_tol_min = -M_PI/6.0 ;
     yaw_tol_max =  M_PI/6.0 ;
     pitch_tol_min = -M_PI/4 ;
     pitch_tol_max = M_PI/4 ;


}

bool FlipHandsPlanner::tearingConstraint(double length, const JointState &js)
{
    KinematicsModel *model = pCtx->getModel() ;

    JointState rs = model->getJointState() ;
    model->setJointState(js) ;

    Affine3d p1 = model->getWorldTransform("r1_ee") ;
    Affine3d p2 = model->getWorldTransform("r2_ee") ;

    double d = (p1.translation() - p2.translation()).norm() ;

   // return d >= length * min_dist_perc && d <= length  ;

    model->setJointState(rs);

    return d <= length ;

}

bool FlipHandsPlanner::plan(trajectory_msgs::JointTrajectory &traj)
{

    JointSpacePlanner planner(pCtx) ;

    planner.setTimeOut(5);

    // measure the distance between hands and use it to setup the non-tearing constraint

    KinematicsModel *model = pCtx->getModel() ;

    Affine3d p1 = model->getWorldTransform("r1_ee") ;
    Affine3d p2 = model->getWorldTransform("r2_ee") ;

    double d = (p1.translation() - p2.translation()).norm() ;

    planner.addStateValidityChecker(boost::bind(&FlipHandsPlanner::tearingConstraint, this, d, _1));

    JointTrajectory traj_ ;

    bool rplan ;




    if ( arm == "r1")
    {
        GoalRegionPtr rg(new FlipHandsGoalRegion(this, p1, p2)) ;

        rplan = planner.solve(rg, traj_) ;
   }
   else
    {
        GoalRegionPtr rg(new FlipHandsGoalRegion(this, p2, p1)) ;

        rplan = planner.solve(rg, traj_) ;

    }


    if ( rplan )
    {
 //   traj.completeTrajectory(kmodel.getJointState()) ;

        JointState rs = kmodel.getJointState() ;

        trajectory_msgs::JointTrajectory msg = traj_.toMsg(10) ;

        filterTrajectory("arms", msg, traj) ;
    }

    return rplan ;

}


























}
