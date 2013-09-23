#include <robot_helpers/Planner.h>
#include <robot_helpers/Geometry.h>
#include <robot_helpers/JointTrajectory.h>

#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/PathGeometric.h>
#include <ompl/base/StateSpace.h>
#include <ompl/base/ScopedState.h>
#include <ompl/base/State.h>
#include <ompl/base/goals/GoalStates.h>
#include <ompl/base/goals/GoalLazySamples.h>
#include <ompl/base/ProblemDefinition.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/PathGeometric.h>

#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/planners/rrt/LazyRRT.h>
#include <ompl/geometric/planners/rrt/pRRT.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>

#include <ompl/geometric/planners/kpiece/KPIECE1.h>
#include <ompl/geometric/planners/kpiece/BKPIECE1.h>
#include <ompl/geometric/planners/kpiece/LBKPIECE1.h>

#include <ompl/geometric/planners/sbl/SBL.h>
#include <ompl/geometric/planners/sbl/pSBL.h>

#include <boost/thread/mutex.hpp>


using namespace std ;
using namespace Eigen ;

namespace robot_helpers {

////////////////////////////////////////////////////////////////////////////////

void rpy_to_quat(double roll, double pitch, double yaw, double &qx, double &qy, double &qz, double &qw)
{
    double phi, the, psi;
    phi = roll / 2.0;
    the = pitch / 2.0;
    psi = yaw / 2.0;

    qx = sin(phi) * cos(the) * cos(psi) - cos(phi) * sin(the) * sin(psi);
    qy = cos(phi) * sin(the) * cos(psi) + sin(phi) * cos(the) * sin(psi);
    qz = cos(phi) * cos(the) * sin(psi) - sin(phi) * sin(the) * cos(psi);
    qw = cos(phi) * cos(the) * cos(psi) + sin(phi) * sin(the) * sin(psi);

    double sc = sqrt(qx * qx + qy * qy + qz * qz + qw * qw) ;

    qx /= sc ; qy /= sc ; qz /= sc ; qw /= sc ;
}

double NormalizeCircularAngle(double theta, double min_, double max_)
{
    if (theta < min_) {
        double range = max_ - min_;

        assert(range > 0) ;
        theta += range;

        while (theta < min_) {
            theta += range;
        }
    }
    else if (theta > max_) {
        double range = max_ - min_ ;
        assert( range > 0 ) ;
        theta -= range;
        while (theta > max_) {
            theta -= range;
        }
    }
    return theta;
}

void poseToXYZRPY(const Affine3d &pose, double &X, double &Y, double &Z, double &roll, double &pitch, double &yaw)
{
    robot_helpers::rpyFromQuat(Quaterniond(pose.rotation()), roll, pitch, yaw ) ;

    Vector3d p = pose.translation() ;

    roll = NormalizeCircularAngle(roll, -M_PI, M_PI) ;
    pitch = NormalizeCircularAngle(pitch, -M_PI, M_PI) ;
    yaw = NormalizeCircularAngle(yaw, -M_PI, M_PI) ;

    X = p.x() ; Y = p.y() ; Z = p.z() ;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

ompl::base::StateSpacePtr createOmplStateSpace(PlanningContext *manip)
{
    using namespace ompl::base ;

    StateSpacePtr ompl_state_space;

    const vector<string> &chain= manip->getJoints() ;
    KinematicsModel *model = manip->getModel() ;

    int nDOFs = chain.size() ;

    RealVectorBounds real_vector_bounds(0) ;
    RealVectorStateSpace *state_space = new RealVectorStateSpace(nDOFs) ;

    int dim = 0 ;

    for(int i=0 ; i<chain.size() ; i++ )
    {
        double lower, upper ;
        model->getLimits(chain[i], lower, upper) ;

        real_vector_bounds.low.push_back(lower);
        real_vector_bounds.high.push_back(upper);

        state_space->setDimensionName(dim, chain[i])  ;

        ++dim ;
    }

    state_space->setBounds(real_vector_bounds);
    ompl_state_space.reset(state_space);

    return ompl_state_space ;
}

ompl::base::StateSpacePtr createOmplTaskStateSpace(const TaskSpace *ts)
{
    using namespace ompl::base ;

    StateSpacePtr ompl_state_space;

    RealVectorBounds real_vector_bounds(0) ;

    int dim = ts->getDimension() ;

    RealVectorStateSpace *state_space = new RealVectorStateSpace(ts->getDimension()) ;

    for(int i=0 ; i<dim ; i++ )
    {

        double lower = ts->getLowerLimit(i) ;
        double upper = ts->getUpperLimit(i) ;

        real_vector_bounds.low.push_back(lower);
        real_vector_bounds.high.push_back(upper);
    }

    state_space->setBounds(real_vector_bounds);
    ompl_state_space.reset(state_space);

    return ompl_state_space ;
}

void setOmplState(const ompl::base::StateSpacePtr &ompl_state_space,
                  ompl::base::RealVectorStateSpace::StateType &ompl_state,
             const JointState &state)
{
    using namespace ompl::base ;

    RealVectorStateSpace *state_space =
            dynamic_cast<RealVectorStateSpace *>(ompl_state_space.get()) ;

    map<string, double>::const_iterator it = state.joint_values.begin() ;

    for( ; it != state.joint_values.end() ; ++it )
    {
        const string &name = (*it).first ;

        int idx = state_space->getDimensionIndex(name) ;

        if ( idx == -1 ) continue ;

        ompl_state[idx] = (*it).second ;

    }
}

void setOmplTaskState(const ompl::base::StateSpacePtr &ompl_state_space,
                  ompl::base::ScopedState<ompl::base::RealVectorStateSpace> &ompl_state,
             const std::vector<double> &state)
{
    using namespace ompl::base ;

    RealVectorStateSpace *state_space =
            dynamic_cast<RealVectorStateSpace *>(ompl_state_space.get()) ;

    for( int i=0 ; i<state.size() ; i++ )
    {
        ompl_state[i] = state[i] ;
    }


}


void getOmplState(const ompl::base::StateSpacePtr &ompl_state_space,
             const ompl::base::RealVectorStateSpace::StateType *ompl_state,
             JointState &state)
{
    using namespace ompl::base ;

    RealVectorStateSpace *state_space =
            dynamic_cast<RealVectorStateSpace *>(ompl_state_space.get()) ;

    for( int i=0 ; i<state_space->getDimension() ; i++ )
    {
        const string &name = state_space->getDimensionName(i) ;

        double dof = (*ompl_state)[i] ;
        state.joint_values[name] = dof ;
    }
}

void getOmplTaskState(const ompl::base::StateSpacePtr &ompl_state_space,
             const ompl::base::RealVectorStateSpace::StateType *ompl_state,
             std::vector<double> &state)
{
    using namespace ompl::base ;

    RealVectorStateSpace *state_space =
            dynamic_cast<RealVectorStateSpace *>(ompl_state_space.get()) ;

    for( int i=0 ; i<state_space->getDimension() ; i++ )
    {
        state.push_back((*ompl_state)[i]) ;
    }
}

void getOmplTrajectory(const ompl::geometric::PathGeometric &path,
                       const ompl::base::StateSpacePtr &ompl_state_space,
                       JointTrajectory &joint_trajectory)
{
    using namespace ompl::base ;

    unsigned int num_points = path.getStateCount();

    double t = 0.0, tstep = 1.0/(num_points - 1) ;


    for(int i=0 ; i<num_points ; i++)
    {
        const RealVectorStateSpace::StateType *state =
                path.getState(i)->as<RealVectorStateSpace::StateType>() ;

        JointState js ;
        getOmplState(ompl_state_space, state, js) ;

        joint_trajectory.addPoint(t, js) ;

        t += tstep ;

    }



}

bool getOmplTaskTrajectory(const ompl::geometric::PathGeometric &path,
                       const ompl::base::StateSpacePtr &ompl_state_space,
                       const TaskSpace *ts,
                       const PlanningContext *group,
                       JointTrajectory &joint_trajectory)
{
    using namespace ompl::base ;

    unsigned int num_points = path.getStateCount();

    double t = 0.0, tstep = 1.0/(num_points - 1) ;

    JointState previous_state = group->getModel()->getJointState() ;

    for(int i=0 ; i<num_points ; i++)
    {
        const RealVectorStateSpace::StateType *state =
                path.getState(i)->as<RealVectorStateSpace::StateType>() ;

        std::vector<double> tsv ;
        getOmplTaskState(ompl_state_space, state, tsv) ;

        // convert this state to pose and find an IK solution
        vector<Affine3d> pose ;
        ts->taskSpaceToPose(tsv, pose) ;

        JointState solution ;

        if ( !group->solve(pose, previous_state, solution) ) {
            return false ;
        }
        else {
            previous_state = solution ;
            joint_trajectory.addPoint(t, previous_state) ;
        }


        t += tstep ;

    }

    return true ;
}

///////////////////////////////////////////////////////////////////////////////

class OmplJointGoalSampler {
    public:

    OmplJointGoalSampler(const ompl::base::SpaceInformationPtr &space_information,
                   const ompl::base::ProblemDefinitionPtr &pd,
                   GoalRegionPtr goal,
                   const PlanningContextPtr &manip) ;

    bool sampleGoal(const ompl::base::GoalLazySamples *gls, ompl::base::State *state) ;
    bool randomSample(JointState &sample) ;

    int sampleNum ;
    const PlanningContextPtr manip ;
    GoalRegionPtr goal ;
    vector<string> joints ;
    JointState cs ;

    const ompl::base::SpaceInformationPtr &si ;
    const ompl::base::ProblemDefinitionPtr &pd ;
    int max_sample_count_ ;



};

OmplJointGoalSampler::OmplJointGoalSampler(const ompl::base::SpaceInformationPtr &space_information_,
                           const ompl::base::ProblemDefinitionPtr &pd_,
                           GoalRegionPtr goal_,
                           const PlanningContextPtr &manip_): manip(manip_), goal(goal_),
    si(space_information_), sampleNum(0), max_sample_count_(10000), pd(pd_) {

    joints = manip->getJoints() ;
    manip->getJointState(cs);
}

bool OmplJointGoalSampler::randomSample(JointState &sample)
{
    vector<double> sv ;
    vector<Affine3d> pose ;

    goal->sample(sv) ;

    for( int i=0, k=0; i<manip->getManipulators() ; i++ )
    {
        double qx, qy, qz, qw ;
        double x = sv[k++], y = sv[k++], z = sv[k++], roll = sv[k++], pitch = sv[k++], yaw = sv[k++] ;

        rpy_to_quat(roll, pitch, yaw, qx, qy, qz, qw) ;

        Quaterniond rot(qw, qx, qy, qz) ;
        Translation3d trans(x, y, z) ;

        pose.push_back( trans * rot.toRotationMatrix() ) ;
    }

    bool res = manip->solve(pose, cs, sample) ;

    return res ;
}


bool OmplJointGoalSampler::sampleGoal(const ompl::base::GoalLazySamples *gls, ompl::base::State *state)
{
    using namespace ompl::base ;

    const StateSpacePtr &ompl_state_space = si->getStateSpace() ;
    RealVectorStateSpace::StateType *ompl_state = static_cast<RealVectorStateSpace::StateType *>(state) ;

    while (sampleNum < max_sample_count_)
    {
        JointState sample ;

        sampleNum ++ ;

        if ( ! randomSample(sample) ) continue ;
        else {

            setOmplState(ompl_state_space, *ompl_state, sample) ;

            break ;
        }

   }

    return sampleNum < max_sample_count_ ; /*&& !pd->hasSolution()*/


}

class OmplValidityChecker: public ompl::base::StateValidityChecker
{
public:

    OmplValidityChecker(const ompl::base::SpaceInformationPtr &si,
                        const PlanningContextPtr manip_, const vector<JointStateValidityChecker> &checkers_): StateValidityChecker(si), manip(manip_),
        ompl_state_space(si->getStateSpace()), checkers(checkers_)
    {

    }

    bool isValid(const ompl::base::State *state) const ;

private:

    const ompl::base::StateSpacePtr &ompl_state_space ;
    PlanningContextPtr manip ;
    const vector<JointStateValidityChecker> &checkers ;
};

bool OmplValidityChecker::isValid(const ompl::base::State *state) const
{
    using namespace ompl::base ;

    const RealVectorStateSpace::StateType *ompl_state =
            dynamic_cast<const RealVectorStateSpace::StateType *>(state) ;

    JointState js ;
    getOmplState(ompl_state_space, ompl_state, js) ;

    KinematicsModel *model = manip->getModel() ;

    bool res = model->isStateValid(js) ;

    if ( !res ) return false ;

    for( int i=0 ; i<checkers.size() ; i++ )
    {
        if ( !checkers[i](js) ) return false ;
    }

    return true ;

}

////////////////////////////////////////////////////////////////////////////////


bool JointSpacePlanner::solve(GoalRegionPtr &goal_,
                                 JointTrajectory &traj)
{
    using namespace ompl::base ;

  //  ompl::msg::noOutputHandler() ;

    StateSpacePtr ompl_state_space = createOmplStateSpace(pctx.get()) ;

    // create a simple setup and set start and goal state
    ompl::geometric::SimpleSetupPtr ompl_planner_setup ;

    ompl_planner_setup.reset(new ompl::geometric::SimpleSetup(ompl_state_space)) ;

    SpaceInformationPtr si = ompl_planner_setup->getSpaceInformation() ;

    si->setStateValidityChecker(StateValidityCheckerPtr(new OmplValidityChecker(si, pctx, checkers)));

    // use the current joint state of the manipulator as the start state

    ScopedState<RealVectorStateSpace> start_state(ompl_state_space) ;

    JointState js ;
    pctx->getJointState(js) ;
    setOmplState(ompl_state_space, *start_state.get(), js) ;

    // set the goal region
    ompl::base::GoalPtr goal;

    boost::shared_ptr<OmplJointGoalSampler> goal_sampler(new OmplJointGoalSampler(si, ompl_planner_setup->getProblemDefinition(), goal_,  pctx)) ;

    goal.reset(new ompl::base::GoalLazySamples(si, boost::bind(&OmplJointGoalSampler::sampleGoal, goal_sampler,_1,_2)));

    ompl_planner_setup->setStartState(start_state);
    ompl_planner_setup->setGoal(goal) ;

    // set the planner

    PlannerPtr ompl_planner ;

    switch ( alg )
    {

    case RRT:
        ompl_planner.reset(new ompl::geometric::RRT(si)) ;
        break ;
    case pRRT:
        ompl_planner.reset(new ompl::geometric::pRRT(si)) ;
        break ;
    case LazyRRT:
        ompl_planner.reset(new ompl::geometric::LazyRRT(si)) ;
        break ;
    case RRTConnect:
        ompl_planner.reset(new ompl::geometric::RRTConnect(si)) ;
        break ;
    case KPIECE:
        ompl_planner.reset(new ompl::geometric::KPIECE1(si)) ;
        break ;
    case BKPIECE:
        ompl_planner.reset(new ompl::geometric::BKPIECE1(si)) ;
        break ;
    case LBKPIECE:
        ompl_planner.reset(new ompl::geometric::LBKPIECE1(si)) ;
        break ;
    case SBL:
        ompl_planner.reset(new ompl::geometric::SBL(si)) ;
        break ;
    case pSBL:
        ompl_planner.reset(new ompl::geometric::pSBL(si)) ;
        break ;
    default:
        return false ;
    }


    //planner->setGoalBias(0.0) ;
    //planner->setRange(0.0) ;

    ompl_planner_setup->setPlanner(ompl_planner) ;

    // solve problem

    PlannerStatus res = ompl_planner_setup->solve(time_out);



    if ( res  )
    {
        // get solution path

        ompl::geometric::PathGeometric &path = ompl_planner_setup->getSolutionPath() ;
        ompl::geometric::PathSimplifierPtr &pathSimplifier =
            ompl_planner_setup->getPathSimplifier() ;

        // simplify path
        pathSimplifier->simplifyMax(path);

        getOmplTrajectory(path, ompl_state_space, traj) ;

        return true ;

    }

    return false ;

}

/////////////////////////////////////////////////////////////////////////////////////////////


SimplePoseGoal::SimplePoseGoal(const Affine3d &pose_, double small_): pose(pose_)
{
    goal_tolerance_x = goal_tolerance_y = goal_tolerance_z = small_ ;

    roll_delta_minus = roll_delta_plus = small_ ;
    yaw_delta_minus = yaw_delta_plus = small_ ;
    pitch_delta_minus = pitch_delta_plus = small_ ;
}

void SimplePoseGoal::computeBounds()
{
    double x, y, z, roll, pitch, yaw ;

    poseToXYZRPY(pose, x, y, z, roll, pitch, yaw) ;

    double xmin = x - fabs(goal_tolerance_x) ;
    double ymin = y - fabs(goal_tolerance_y) ;
    double zmin = z - fabs(goal_tolerance_z) ;
    double xmax = x + fabs(goal_tolerance_x) ;
    double ymax = y + fabs(goal_tolerance_y) ;
    double zmax = z + fabs(goal_tolerance_z) ;

    double roll_min = std::max(roll - fabs(roll_delta_minus), -M_PI) ;
    double roll_max = std::min(roll + fabs(roll_delta_plus), M_PI) ;

    double pitch_min = std::max(pitch - fabs(pitch_delta_minus), -M_PI) ;
    double pitch_max = std::min(pitch + fabs(pitch_delta_plus), M_PI) ;

    double yaw_min = std::max(yaw - fabs(yaw_delta_minus), -M_PI) ;
    double yaw_max = std::min(yaw + fabs(yaw_delta_plus), M_PI) ;

    lower_bounds.push_back(xmin) ;
    lower_bounds.push_back(ymin) ;
    lower_bounds.push_back(zmin) ;
    lower_bounds.push_back(roll_min) ;
    lower_bounds.push_back(pitch_min) ;
    lower_bounds.push_back(yaw_min) ;

    upper_bounds.push_back(xmax) ;
    upper_bounds.push_back(ymax) ;
    upper_bounds.push_back(zmax) ;
    upper_bounds.push_back(roll_max) ;
    upper_bounds.push_back(pitch_max) ;
    upper_bounds.push_back(yaw_max) ;
}

ompl::RNG g_rng ;

void SimplePoseGoal::sample(vector<double> &xyz_rpy)
{
    if ( lower_bounds.empty() ) computeBounds() ;

    double X = g_rng.uniformReal(lower_bounds[0], upper_bounds[0]) ;
    double Y = g_rng.uniformReal(lower_bounds[1], upper_bounds[1]) ;
    double Z = g_rng.uniformReal(lower_bounds[2], upper_bounds[2]) ;
    double Roll  = g_rng.uniformReal(lower_bounds[3], upper_bounds[3]) ;
    double Pitch = g_rng.uniformReal(lower_bounds[4], upper_bounds[4]) ;
    double Yaw   = g_rng.uniformReal(lower_bounds[5], upper_bounds[5]) ;

    xyz_rpy.push_back(X) ;
    xyz_rpy.push_back(Y) ;
    xyz_rpy.push_back(Z) ;
    xyz_rpy.push_back(Roll) ;
    xyz_rpy.push_back(Pitch) ;
    xyz_rpy.push_back(Yaw) ;
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////


class OmplTaskValidityChecker: public ompl::base::StateValidityChecker
{
public:

    OmplTaskValidityChecker(const ompl::base::SpaceInformationPtr &si,
                            const PlanningContextPtr &manip_, const TaskSpace *ts_): StateValidityChecker(si), manip(manip_), ts(ts_),
        ompl_state_space(si->getStateSpace())
    {

    }

    bool isValid(const ompl::base::State *state) const ;

private:

    const ompl::base::StateSpacePtr &ompl_state_space ;
    PlanningContextPtr manip ;
    const TaskSpace *ts ;
    mutable boost::mutex mtx ;
};

bool OmplTaskValidityChecker::isValid(const ompl::base::State *state) const
{
    using namespace ompl::base ;

    const RealVectorStateSpace::StateType *ompl_state =
            dynamic_cast<const RealVectorStateSpace::StateType *>(state) ;

    std::vector<double> tsv ;

    getOmplTaskState(ompl_state_space, ompl_state, tsv) ;

    vector<Affine3d> pose ;
    ts->taskSpaceToPose(tsv, pose) ;

    KinematicsModel *model = manip->getModel() ;

    JointState ik_solution ;


        boost::lock_guard<boost::mutex> lock(mtx);

    bool res = manip->solve(pose, model->getJointState(), ik_solution ) ;


    return res ;

}


class OmplTaskGoalSampler {
    public:

    OmplTaskGoalSampler(const ompl::base::SpaceInformationPtr &space_information,
                   const ompl::base::ProblemDefinitionPtr &pd,
                   GoalRegion *goal,
                   const TaskSpace *ts,
                   const PlanningContextPtr &manip) ;

    bool sampleGoal(const ompl::base::GoalLazySamples *gls, ompl::base::State *state) ;
    void randomSample(std::vector<double> &sample) ;

    int sampleNum ;
    PlanningContextPtr manip ;
    const TaskSpace *ts ;
    GoalRegion *goal ;


    const ompl::base::SpaceInformationPtr &si ;
    const ompl::base::ProblemDefinitionPtr &pd ;
    int max_sample_count_ ;



};

OmplTaskGoalSampler::OmplTaskGoalSampler(const ompl::base::SpaceInformationPtr &space_information_,
                           const ompl::base::ProblemDefinitionPtr &pd_,
                           GoalRegion *goal_,
                           const TaskSpace *ts_,
                           const PlanningContextPtr &manip_): manip(manip_), goal(goal_),
    ts(ts_), si(space_information_), sampleNum(0), max_sample_count_(10000), pd(pd_) { }

void OmplTaskGoalSampler::randomSample(std::vector<double> &sample)
{
    vector<double> sv ;
    vector<Affine3d> pose ;

    goal->sample(sv) ;

    for( int i=0, k=0; i<manip->getManipulators() ; i++ )
    {
        double qx, qy, qz, qw ;
        double x = sv[k++], y = sv[k++], z = sv[k++], roll = sv[k++], pitch = sv[k++], yaw = sv[k++] ;

        rpy_to_quat(roll, pitch, yaw, qx, qy, qz, qw) ;

        Quaterniond rot(qw, qx, qy, qz) ;
        Translation3d trans(x, y, z) ;

        pose.push_back( trans * rot.toRotationMatrix() ) ;
    }

    ts->poseToTaskSpace(pose, sample) ;

}

bool OmplTaskGoalSampler::sampleGoal(const ompl::base::GoalLazySamples *gls, ompl::base::State *state)
{
    using namespace ompl::base ;

    const StateSpacePtr &ompl_state_space = si->getStateSpace() ;

    while (sampleNum < max_sample_count_)
    {
        std::vector<double> sample ;

        randomSample(sample) ;

        sampleNum ++ ;

        double *vals = static_cast<RealVectorStateSpace::StateType*>(state)->values ;

        for(int i=0 ; i<sample.size() ; i++ ) vals[i] = sample[i] ;

        RealVectorStateSpace *state_space =
                dynamic_cast<RealVectorStateSpace *>(ompl_state_space.get()) ;

        if ( !ompl_state_space->satisfiesBounds(state) ) continue ;
        else break ;

   }

    return sampleNum < max_sample_count_ /*&& !pd->hasSolution() */;


}

bool TaskSpacePlanner::solve(GoalRegion &goal_, const TaskSpace &ts, JointTrajectory &traj)
{
    using namespace ompl::base ;

    StateSpacePtr ompl_state_space = createOmplTaskStateSpace(&ts) ;

    // create a simple setup and set start and goal state
    ompl::geometric::SimpleSetupPtr ompl_planner_setup ;

    ompl_planner_setup.reset(new ompl::geometric::SimpleSetup(ompl_state_space)) ;

    SpaceInformationPtr si = ompl_planner_setup->getSpaceInformation() ;

    si->setStateValidityChecker(StateValidityCheckerPtr(new OmplTaskValidityChecker(si, pctx, &ts)));

    // use the current joint state of the manipulator as the start state

    ScopedState<RealVectorStateSpace> start_state(ompl_state_space) ;

    vector<Affine3d> pose ;

    for( int i=0 ; i<pctx->getManipulators() ; i++ )
    {
        std::string ee = pctx->getEndEffector(i) ;
        Affine3d pose_ = pctx->getModel()->getWorldTransform(ee) ;
        pose.push_back(pose_) ;
    }

    std::vector<double> start_state_vec ;
    ts.poseToTaskSpace(pose, start_state_vec) ;

    setOmplTaskState(ompl_state_space, start_state, start_state_vec) ;

    // set the goal region
    ompl::base::GoalPtr goal;

    OmplTaskGoalSampler goal_sampler(si, ompl_planner_setup->getProblemDefinition(), &goal_, &ts, pctx) ;

    goal.reset(new ompl::base::GoalLazySamples(si, boost::bind(&OmplTaskGoalSampler::sampleGoal, &goal_sampler,_1,_2)));

    ompl_planner_setup->setStartState(start_state);
    ompl_planner_setup->setGoal(goal) ;

    // set the planner

    PlannerPtr ompl_planner ;

    switch ( alg )
    {

    case RRT:
        ompl_planner.reset(new ompl::geometric::RRT(si)) ;
        break ;
    case pRRT:
        ompl_planner.reset(new ompl::geometric::pRRT(si)) ;
        break ;
    case LazyRRT:
        ompl_planner.reset(new ompl::geometric::LazyRRT(si)) ;
        break ;
    case RRTConnect:
        ompl_planner.reset(new ompl::geometric::RRTConnect(si)) ;
        break ;
    case KPIECE:
        ompl_planner.reset(new ompl::geometric::KPIECE1(si)) ;
        break ;
    case BKPIECE:
        ompl_planner.reset(new ompl::geometric::BKPIECE1(si)) ;
        break ;
    case LBKPIECE:
        ompl_planner.reset(new ompl::geometric::LBKPIECE1(si)) ;
        break ;
    case SBL:
        ompl_planner.reset(new ompl::geometric::SBL(si)) ;
        break ;
    case pSBL:
        ompl_planner.reset(new ompl::geometric::pSBL(si)) ;
        break ;
    default:
        return false ;
    }


    //planner->setGoalBias(0.0) ;
    //planner->setRange(0.0) ;

    ompl_planner_setup->setPlanner(ompl_planner) ;

    // solve problem

    PlannerStatus res = ompl_planner_setup->solve(time_out);

    if ( res  )
    {
        // get solution path

        ompl::geometric::PathGeometric &path = ompl_planner_setup->getSolutionPath() ;
        ompl::geometric::PathSimplifierPtr &pathSimplifier =
            ompl_planner_setup->getPathSimplifier() ;

        // simplify path
        pathSimplifier->simplifyMax(path);

        return getOmplTaskTrajectory(path, ompl_state_space, &ts, pctx.get(), traj) ;
    }

    return false ;
}

}
