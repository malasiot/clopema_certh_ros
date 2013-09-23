#ifndef __PLANNING_CONTEXT_H__
#define __PLANNING_CONTEXT_H__

#include <Eigen/Geometry>
#include <robot_helpers/KinematicsInterface.h>

namespace robot_helpers {

class PlanningContext {
public:
    PlanningContext(const std::string &group, KinematicsModel *kmodel): group_(group), kmodel_(kmodel) {}
     ~PlanningContext() {}

    std::vector<std::string> getJoints() const { return kmodel_->getJoints(group_); }

    KinematicsModel *getModel() const { return kmodel_ ; }

    void getJointState(JointState &js) const {
        js = kmodel_->getJointState() ;
    }

    virtual std::string getEndEffector(int nanip) const = 0 ;

    virtual int getManipulators() const = 0 ;

    virtual bool solve(const std::vector<Eigen::Affine3d> &pose, const JointState &cur, JointState &solutions) const = 0;

protected:
    KinematicsModel *kmodel_ ;
    std::string group_ ;
};

typedef boost::shared_ptr<PlanningContext> PlanningContextPtr ;

// Planning context for single arm

class PlanningContextSingle: public PlanningContext {
public:
    PlanningContextSingle(const std::string &group, KinematicsModel *kmodel,
                          IKSolver *solver, const std::string &ee):
        PlanningContext(group, kmodel), solver_(solver), ee_(ee) {
    }

    int getManipulators() const { return 1 ; }
    std::string getEndEffector(int manip) const { return ee_ ; }

    bool solve(const std::vector<Eigen::Affine3d> &pose, const JointState &cur, JointState &solution) const {
        bool res = solver_->solveIK(ee_, pose[0].translation(), Eigen::Quaterniond(pose[0].rotation()), cur, solution) ;
        if ( !res ) return false ;

        return kmodel_->isStateValid(solution) ;

    }

protected:

    std::string ee_ ;
    IKSolver *solver_ ;
};

// Planning context for dual arm

class PlanningContextDual: public PlanningContext {
public:
    PlanningContextDual(const std::string &group, KinematicsModel *kmodel, const IKSolverPtr &solver1, const std::string &ee1,
                        const IKSolverPtr solver2,  const std::string &ee2 ):
        PlanningContext(group, kmodel), solver1_(solver1), ee1_(ee1),
            solver2_(solver2), ee2_(ee2)
    {
        solver1_->setKinematicModel(kmodel);
        solver2_->setKinematicModel(kmodel);
    }

    int getManipulators() const { return 2 ; }
    std::string getEndEffector(int manip) const { return (manip == 0 ) ? ee1_ : ee2_ ; }

    bool solve(const std::vector<Eigen::Affine3d> &pose, const JointState &cur, JointState &solution) const {

        JointState solution1, solution2 ;

        bool res1 = solver1_->solveIK(ee1_, pose[0].translation(), Eigen::Quaterniond(pose[0].rotation()), cur, solution1) ;
        bool res2 = solver2_->solveIK(ee2_, pose[1].translation(), Eigen::Quaterniond(pose[1].rotation()), cur, solution2) ;

        if ( res1 && res2 ) {
            solution = JointState::merged(solution1, solution2) ;

            return kmodel_->isStateValid(solution) ;
        }
        else return false ;

    }

protected:

    std::string ee1_, ee2_ ;
    boost::shared_ptr<IKSolver> solver1_, solver2_ ;
};

class PlanningContextDualDefault: public PlanningContextDual {
public:
    PlanningContextDualDefault(KinematicsModel *kmodel):
        PlanningContextDual("arms", kmodel, IKSolverPtr(new MA1400_R1_IKSolver), "r1_ee",  IKSolverPtr(new MA1400_R1_IKSolver), "r2_ee")
    {

    }

};


//////////////////////////////////////////////////////////////////////////////////////////////////////





}


#endif
