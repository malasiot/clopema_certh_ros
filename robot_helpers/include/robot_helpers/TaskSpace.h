#ifndef __ROBOT_HELPERS_TASK_SPACE_H__
#define __ROBOT_HELPERS_TASK_SPACE_H__

#include <Eigen/Geometry>
#include <vector>

namespace robot_helpers {

class TaskSpace
{
public:
    TaskSpace() {}

    // get the dimension of the task space
    virtual int getDimension() const = 0 ;

    // get upper-lower limits of each dimension
    virtual double getUpperLimit(int dim) const = 0 ;
    virtual double getLowerLimit(int dim) const = 0 ;

    // transform a state from task space to a pose of the end-effector(s) in world coordinates
    virtual void taskSpaceToPose(const std::vector<double> &state, std::vector<Eigen::Affine3d> &pose) const = 0;
    // transform pose of the end-effector(s) to state in task space
    virtual void poseToTaskSpace(const std::vector<Eigen::Affine3d> &pose, std::vector<double> &state) const = 0;

};

// This is the most common task space defined by the pose of the end-effector

class RPY_XYZ_TaskSpace: public TaskSpace {
    public:

    RPY_XYZ_TaskSpace() ;

    virtual int getDimension() const ;

    // get upper-lower limits of each dimension
    virtual double getUpperLimit(int dim) const ;
    virtual double getLowerLimit(int dim) const ;

    // transform a state from task space to a pose of the end-effector in world coordinates
    virtual void taskSpaceToPose(const std::vector<double> &state, std::vector<Eigen::Affine3d> &pose) const ;
    // transform pose of the end-effector to state in task space
    virtual void poseToTaskSpace(const std::vector<Eigen::Affine3d> &pose, std::vector<double> &state) const ;

private:

    double upper[6], lower[6] ;

};

// This may be used to  move the tip linearly from the current position to a target position while keeping the orientation within limits

class MoveTo_TaskSpace: public TaskSpace {
    public:

    /*
     * pose:    The current pose of the end-effector
     * dp:      The displacement of the end-effector
     * pos_tol: This is the positional tolerance of the end-effector with respect to the line between the start and end point
     * rpy_minus, rpy_plus: Negative and positive offset of end-effector orientation with respect to the current orientation
    */

    MoveTo_TaskSpace(const Eigen::Affine3d &pose, const Eigen::Vector3d &dp, double pos_tol,
                     const Eigen::Vector3d &rpy_minus, const Eigen::Vector3d &rpy_plus) ;

    virtual int getDimension() const { return 6 ; }

    virtual double getUpperLimit(int dim) const { return upper[dim]; }
    virtual double getLowerLimit(int dim) const { return lower[dim]; }

    virtual void taskSpaceToPose(const std::vector<double> &state, std::vector<Eigen::Affine3d> &pose) const ;
    virtual void poseToTaskSpace(const std::vector<Eigen::Affine3d> &pose, std::vector<double> &state) const ;

private:

    double upper[6], lower[6] ;
    Eigen::Vector3d c0, a0 ;
    Eigen::Matrix3d frame, iframe ;
    double travel_dist ;

};


// This builds a dual arm taskspace by concatenating states of individual arms

class TaskSpaceDualComposite: public TaskSpace
{
public:
    TaskSpaceDualComposite(TaskSpace *left_arm_task_space, TaskSpace *right_arm_task_space): lspace(left_arm_task_space), rspace(right_arm_task_space) {} ;

    // get the dimension of the task space
    virtual int getDimension() const { return lspace->getDimension() + rspace->getDimension() ; }
    virtual double getUpperLimit(int dim) const { return (dim < lspace->getDimension() ) ? lspace->getUpperLimit(dim) : rspace->getUpperLimit(dim - lspace->getDimension()) ; }
    virtual double getLowerLimit(int dim) const { return (dim < lspace->getDimension() ) ? lspace->getLowerLimit(dim) : rspace->getLowerLimit(dim - lspace->getDimension()) ; }
    virtual void taskSpaceToPose(const std::vector<double> &state, std::vector<Eigen::Affine3d> &pose) const ;
    virtual void poseToTaskSpace(const std::vector<Eigen::Affine3d> &pose, std::vector<double> &state) const ;

protected:
    TaskSpace *lspace, *rspace ;
};
}
#endif
