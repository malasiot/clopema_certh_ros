#ifndef __ROBOT_HELPERS_PLANNER_H__
#define __ROBOT_HELPERS_PLANNER_H__

#include <string>
#include <robot_helpers/KinematicsInterface.h>
#include <robot_helpers/PlanningContext.h>
#include <robot_helpers/JointTrajectory.h>
#include <robot_helpers/GoalRegion.h>
#include <robot_helpers/TaskSpace.h>

#include <Eigen/Geometry>

namespace robot_helpers {

class PlannerBase {

public:
    enum Algorithm {
        KPIECE, BKPIECE, LBKPIECE, SBL, pSBL, EST, RRT, RRTConnect, LazyRRT, pRRT, PRM, PRMStar
    } ;

    PlannerBase(const PlanningContextPtr &pc): pctx(pc), alg(RRT), time_out(5.0) {

    }

    void setAlgorithm(Algorithm alg_) { alg = alg_ ; }
    void setTimeOut(double time_out_) { time_out = time_out_ ; }

protected:

    Algorithm alg ;
    double time_out ;
    PlanningContextPtr pctx ;
};

typedef boost::function<bool (const JointState &)> JointStateValidityChecker ;

class JointSpacePlanner: public PlannerBase {

public:

    JointSpacePlanner(const PlanningContextPtr &ctx): PlannerBase(ctx) {}

    // add a function to be called for checking the validity of a state beyond collision detection

    void addStateValidityChecker(const JointStateValidityChecker &func) {
        checkers.push_back(func) ;
    }

    // find a trajectory that brings the end-effector in one of the specified poses
    bool solve(GoalRegionPtr &goal, JointTrajectory &traj) ;



private:

    std::vector<JointStateValidityChecker> checkers ;
};


// class to perform planning in task space of the end-effector(s)

class TaskSpacePlanner: public PlannerBase {

public:

    TaskSpacePlanner(const PlanningContextPtr &ctx): PlannerBase(ctx) {}

    // find a trajectory that brings the end-effector in the task-space region given by the goal

    bool solve(GoalRegion &goal, const TaskSpace &ts, JointTrajectory &traj) ;
};



} // namespace robot_helpers



#endif
