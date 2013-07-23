#ifndef __KINEMATICS_INTERFACE_H__
#define __KINEMATICS_INTERFACE_H__

#include <kinematics_base/kinematics_base.h>
#include <arm_navigation_msgs/GetPlanningScene.h>
#include <planning_environment/models/model_utils.h>
#include <visualization_msgs/MarkerArray.h>

#include <robot_helpers/Robot.h>
#include <robot_helpers/Utils.h>
#include <robot_helpers/Geometry.h>


namespace robot_helpers {

// helper class to represent joint states
struct JointState {

    JointState() {}
    JointState(const std::vector<std::string> &names, const std::vector<double> &positions) {
        init(names, positions) ;
    }

    JointState(const sensor_msgs::JointState &js) {
        init(js.name, js.position) ;
    }

    std::vector<double> getValues(const std::vector<std::string> &joint_names)  const;

    static JointState fromRobotState() ;

    std::map<std::string, double> joint_values ;

protected:

    void init(const std::vector<std::string> &names, const std::vector<double> &positions) ;

};

// Wrapper for ROS kinematic models and collision detection
class KinematicsModel {
public:

    KinematicsModel() ;

    bool init() ;

    JointState getJointState() const ;
    JointState getJointState(const std::vector<std::string> &joints) ;

    void setJointValue(const std::string &joint, double val) ;
    void setJointState(const JointState &state) ;

    void getLimits(const std::string &joint_, double &lower_, double &upper_) ;

    bool isStateValid() const ;

    void getRobotMarkers(visualization_msgs::MarkerArray &markers) ;

    Eigen::Affine3d getWorldTransform(const std::string &linkName) ;

    ~KinematicsModel() ;

private:

    boost::shared_ptr<planning_environment::CollisionModels> cm_ ;
    planning_models::KinematicState *state_ ;
    std::vector<std::string> joint_names, link_names ;

};

class IKSolver {

public:
    void setKinematicModel(KinematicsModel *model) { model_ = model ; }

    /**
     * @brief Find a valid IK solution
     * @param tipName the name of the frame for which the pose is specified
     * @param pos pose translation component
     * @param orient pose orientation component
     * @param seed_state a state to provided as a seed to the IK solver
     * @return True if a valid solution was found, false otherwise
     */

    virtual bool solveIK(const std::string &tipName, const Eigen::Vector3d &pos, const Eigen::Quaterniond &orient,
                         const JointState &seed_state, JointState &state) ;

protected:

    bool init(const std::string &groupName, const std::string &instanceName, const std::string &baseLink,  const std::string &tipLink) ;

    void collisionCheck(const geometry_msgs::Pose &ik_pose,  const std::vector<double> &ik_solution, int &error_code);

    void initialPoseCheck(const geometry_msgs::Pose &ik_pose, const std::vector<double> &ik_solution, int &error_code);

    boost::shared_ptr<kinematics::KinematicsBase> solver_ ;
    KinematicsModel *model_ ;
    std::vector<std::string> joint_names_ ;

};


class MA1400_R1_IKSolver: public IKSolver {
public:  MA1400_R1_IKSolver() ;
} ;


class MA1400_R2_IKSolver: public IKSolver {
public:  MA1400_R2_IKSolver() ;
} ;

class MA1400_R1_Xtion_IKSolver: public IKSolver {
public: MA1400_R1_Xtion_IKSolver() ;
} ;

class MA1400_R2_Xtion_IKSolver: public IKSolver {
public:  MA1400_R2_Xtion_IKSolver() ;
} ;

class MA1400_R1_IKFastSolver: public IKSolver {
public:  MA1400_R1_IKFastSolver() ;
} ;

class MA1400_R2_IKFastSolver: public IKSolver {
public:  MA1400_R2_IKFastSolver() ;
} ;

class MA1400_R1_Xtion_IKFastSolver: public IKSolver {
public: MA1400_R1_Xtion_IKFastSolver() ;
} ;

class MA1400_R2_Xtion_IKFastSolver: public IKSolver {
public:  MA1400_R2_Xtion_IKFastSolver() ;
} ;

}

#endif
