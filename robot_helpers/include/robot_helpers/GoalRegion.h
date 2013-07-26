#ifndef __ROBOT_HELPERS_GOAL_REGION_H__
#define __ROBOT_HELPERS_GOAL_REGION_H__

#include <vector>
#include <Eigen/Geometry>

namespace robot_helpers {

class GoalRegion {

public:
    GoalRegion() {};

    // sampling function. for dual arm manipulators the sample for each end-effector are concatenated

    virtual void sample(std::vector<double> &xyz_rpy) = 0;

};


// This class defines a tolerance zone around a desired pose

class SimplePoseGoal: public GoalRegion {

public:
    // The constructor initializes the tolerance values to small_
    // The user has then to expand tolerance for specific dimensions

    SimplePoseGoal(const Eigen::Affine3d &pose, double small_ = 0.01) ;

    virtual void sample(std::vector<double> &xyz_rpy);

    double goal_tolerance_x, goal_tolerance_y, goal_tolerance_z ;
    double roll_delta_minus, roll_delta_plus, yaw_delta_minus, yaw_delta_plus ;
    double pitch_delta_minus, pitch_delta_plus ;

private:

    void computeBounds() ;

    std::vector<double> lower_bounds, upper_bounds ;
    Eigen::Affine3d pose ;

};

// This is the base class for goal regions that constrain the position of the end-effector inside an oriented shape
// The orientation region is a box in roll-yaw-pitch space

class SimpleShapeRegion: public GoalRegion {

public:

    SimpleShapeRegion() ;

    virtual void sample(std::vector<double> &xyz_rpy);

    void setRotation(const Eigen::Quaterniond &rot, double roll_tol, double pitch_tol, double yaw_tol) ;

    double roll_min, roll_max, yaw_min, yaw_max ;
    double pitch_min, pitch_max ;

protected:

    void setShapePose(const Eigen::Vector3d &orig, const Eigen::Vector3d &rpy) ;

    void computeOrientationBounds() ;

    virtual void computePositionBounds() = 0;
    virtual void samplePosition(double &X, double &Y, double &Z) = 0;

    void computeBounds() ;

    std::vector<double> lower_bounds, upper_bounds ;
    Eigen::Affine3d t ;
};

class BoxShapedRegion: public SimpleShapeRegion {

public:
    // The constructor initializes the tolerance values to small_
    // The user has then to expand tolerance for specific dimensions

    BoxShapedRegion(const Eigen::Vector3d &c, const Eigen::Vector3d &sz, const Eigen::Vector3d &rpy) ;

protected:

    virtual void computePositionBounds() ;
    virtual void samplePosition(double &X, double &Y, double &Z) ;

private:

    Eigen::Vector3d orig, rpy, sz ;
};

class CylinderShapedRegion: public SimpleShapeRegion {

public:
    // The constructor initializes the tolerance values to small_
    // The user has then to expand tolerance for specific dimensions

    CylinderShapedRegion(double length, double radius, const Eigen::Vector3d &c, const Eigen::Vector3d &rpy) ;

protected:

    virtual void computePositionBounds() ;
    virtual void samplePosition(double &X, double &Y, double &Z) ;

private:

    Eigen::Vector3d orig, rpy ;
    double len, rad ;
};

// Define a dual planner goal region as the union of regions over each arm.

class GoalDualCompositeRegion: public GoalRegion {
public:

    GoalDualCompositeRegion(GoalRegion *left_arm_region, GoalRegion *right_arm_region): lreg(left_arm_region), rreg(right_arm_region) {}

    virtual void sample(std::vector<double> &xyz_rpy)
    {
        lreg->sample(xyz_rpy) ;
        rreg->sample(xyz_rpy) ;
    }


private:
    GoalRegion *lreg, *rreg ;

};



}



#endif
