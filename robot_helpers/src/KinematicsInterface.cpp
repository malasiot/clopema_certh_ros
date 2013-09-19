#include <robot_helpers/KinematicsInterface.h>

#include <ros/ros.h>
#include <pluginlib/class_loader.h>

#include <tf_conversions/tf_eigen.h>
#include <geometric_shapes/shape_operations.h>


using namespace std ;


//////////////////////////////////////////////////////////////////////////////////////////////////////////
///

namespace robot_helpers {

/////////////////////////////////////////////////////////////////////////////////////////////////////////

void JointState::init(const vector<string> &names, const vector<double> &pos)
{
    assert(names.size() == pos.size()) ;

    for(int i=0 ; i<names.size() ; i++ )
        joint_values[names[i]] = pos[i] ;
}

JointState JointState::fromRobotState()
{
    arm_navigation_msgs::RobotState rs ;

    if ( getRobotState(rs) )
         return JointState(rs.joint_state) ;
    else
        return JointState() ;
}

std::vector<double> JointState::getValues(const std::vector<std::string> &joint_names) const
{
    std::vector<double> res ;
    for(unsigned int i=0 ; i<joint_names.size() ; i++)
    {
        std::map<std::string, double>::const_iterator it = joint_values.find(joint_names[i]) ;
        if ( it != joint_values.end() ) res.push_back((*it).second) ;
    }

    return res ;
}

JointState JointState::merged(const JointState &js1, const JointState &js2)
{
    JointState res ;

    map<string, double>::const_iterator it = js1.joint_values.begin() ;

    for( ; it != js1.joint_values.end() ; ++it )
        res.joint_values.insert(*it) ;

    it = js2.joint_values.begin() ;

    for( ; it != js2.joint_values.end() ; ++it )
        res.joint_values.insert(*it) ;

    return res ;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////

KinematicsModel::KinematicsModel() { }

bool KinematicsModel::init()
{
    ros::service::waitForService("/environment_server/get_planning_scene");

	arm_navigation_msgs::GetPlanningScene planning_scene;
    
    if (!ros::service::call("/environment_server/get_planning_scene", planning_scene)) {
		ROS_ERROR("Can't get planning scene");
		return false ;
	}

	cm_.reset(new planning_environment::CollisionModels("robot_description")) ;

    cm_->getKinematicModel()->getLinkModelNames(link_names) ;
    cm_->getKinematicModel()->getJointModelNames(joint_names) ;

	state_ = cm_->setPlanningScene(planning_scene.response.planning_scene);

    return true ;

}

bool KinematicsModel::init(const arm_navigation_msgs::PlanningScene &planning_scene_diff)
{
    ros::service::waitForService("/environment_server/get_planning_scene");

    arm_navigation_msgs::GetPlanningScene planning_scene;

    planning_scene.request.planning_scene_diff = planning_scene_diff ;

    if (!ros::service::call("/environment_server/get_planning_scene", planning_scene)) {
        ROS_ERROR("Can't get planning scene");
        return false ;
    }

    cm_.reset(new planning_environment::CollisionModels("robot_description")) ;

    cm_->getKinematicModel()->getLinkModelNames(link_names) ;
    cm_->getKinematicModel()->getJointModelNames(joint_names) ;

    state_ = cm_->setPlanningScene(planning_scene.response.planning_scene);

    return true ;

}

void KinematicsModel::setJointState(const JointState &js)
{
	map<string, double> joint_state_map ;

    state_->getKinematicStateValues(joint_state_map) ;

    map<string, double>::const_iterator it = js.joint_values.begin() ;

    for( ; it != js.joint_values.end() ; ++it )
        joint_state_map[(*it).first] = (*it).second ;

	if ( state_->setKinematicState(joint_state_map) )
		state_->updateKinematicLinks();
}


JointState KinematicsModel::getJointState() const
{
    JointState res ;

    state_->getKinematicStateValues(res.joint_values);
    return res ;
}

bool KinematicsModel::isStateValid() const
{
	return ( state_->areJointsWithinBounds(joint_names ) && !cm_->isKinematicStateInCollision(*state_) )  ;
}

bool KinematicsModel::isStateValid(const JointState &js)
{
    JointState cs = getJointState() ;

    setJointState(js) ;
    bool res = isStateValid() ;
    setJointState(cs) ;

    return res ;
}

void KinematicsModel::getLimits(const string &joint_, double &lower_, double &upper_)
{
    pair<double, double> bounds ;

    state_->getJointState(joint_)->getJointModel()->getVariableBounds(joint_, bounds) ;

    lower_ = bounds.first ;
    upper_ = bounds.second ;
}

void KinematicsModel::getRobotMarkers(visualization_msgs::MarkerArray &markers)
{

	std_msgs::ColorRGBA rgba ;
	rgba.r = 1 ;
	rgba.g = 0 ;
    rgba.b = 0 ;
    rgba.a = 0.5 ;


    cm_->getRobotMarkersGivenState(*state_, markers, rgba, "robot_markers", ros::Duration(0.1), &link_names);
}


Eigen::Affine3d KinematicsModel::getWorldTransform(const string &linkName)
{
	const planning_models::KinematicState::LinkState* link_state_ = state_->getLinkState(linkName);
	tf::Transform tr = link_state_->getGlobalLinkTransform() ;

    Eigen::Affine3d etr ;
    tf::TransformTFToEigen(tr, etr) ;
    return etr ;
}

KinematicsModel::~KinematicsModel() {
    if (cm_) cm_->revertPlanningScene(state_);
}

std::vector<std::string> KinematicsModel::getJoints(const std::string &groupName) const
{
    return state_->getJointStateGroup(groupName)->getJointNames() ;

}



//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


class IKSolverLoader {
public:
    IKSolverLoader():  solver_loader("kinematics_base","kinematics::KinematicsBase") {}

    boost::shared_ptr<kinematics::KinematicsBase> load(const string &groupName, const string &instanceName, const string &baseLink,
                                                       const string &tipLink )
    {
        boost::shared_ptr<kinematics::KinematicsBase> solver  ;

        try {
            solver = solver_loader.createInstance(instanceName);

            if ( solver && !solver->initialize( groupName, baseLink, tipLink, 0.01) ) {
                ROS_ERROR("Could not initialize kinematics solver for group %s", groupName.c_str());

            }
        }
        catch(pluginlib::PluginlibException& ex)
        {
            //handle the class failing to load
            ROS_ERROR("The plugin failed to load for some reason. Error: %s", ex.what());
        }

        return solver ;
    }

    pluginlib::ClassLoader<kinematics::KinematicsBase> solver_loader ;

};

static IKSolverLoader loader ;

bool IKSolver::init(const string &groupName, const string &instanceName, const string &baseLink, const string &tipLink)
{

    solver_ = loader.load(groupName, instanceName, baseLink, tipLink) ;

    if ( solver_ )  {
        joint_names_ = solver_->getJointNames() ;
        return true ;
    }
    else return false ;
}


bool IKSolver::solveIK(const string &tipLinkName, const Eigen::Vector3d &pos, const Eigen::Quaterniond &orient, const JointState &cs, JointState &state)
{
	using namespace Eigen ;

    assert(model_) ;

    Affine3d solver_tip_to_pose_tip = model_->getWorldTransform(solver_->getTipName()).inverse() * model_->getWorldTransform(tipLinkName) ;
    Affine3d frame = model_->getWorldTransform(solver_->getBaseName()) ;
    Affine3d pose_ = frame.inverse() *  Translation3d(pos) * orient * solver_tip_to_pose_tip.inverse() ;

    geometry_msgs::Pose pose = eigenPoseToROS(pose_.translation(), Quaterniond(pose_.rotation())) ;

    vector<double> solution ;
    int error_code ;

    vector<double> seed_state_vals = cs.getValues(joint_names_) ;

    if ( solver_->searchPositionIK(pose, seed_state_vals, 1.0, solution,
                                   boost::bind(&IKSolver::initialPoseCheck, this, _1, _2, _3),
                                   boost::bind(&IKSolver::collisionCheck, this, _1, _2, _3),
                                   error_code ) )
	{
        state = JointState(joint_names_, solution) ;
        return true ;
	}
    else return false ;
}


void IKSolver::collisionCheck(const geometry_msgs::Pose &ik_pose,
                    const std::vector<double> &ik_solution,
                    int &error_code)
{


    // save current state
    JointState cs = model_->getJointState() ;

    JointState js(joint_names_, ik_solution) ;
    model_->setJointState(js) ;

    if ( !model_->isStateValid() ) {
        error_code = kinematics::STATE_IN_COLLISION;
    }
    else error_code = kinematics::SUCCESS;

    model_->setJointState(cs) ;
}

void IKSolver::initialPoseCheck(const geometry_msgs::Pose &ik_pose,
                      const std::vector<double> &ik_solution,
                      int &error_code)
{
    error_code = kinematics::SUCCESS;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////

MA1400_R1_IKSolver::MA1400_R1_IKSolver() {
    init("r1_arm", "clopema_ma1400_kinematics/IKKinematicsPlugin", "r1_link_1", "r1_tip_link") ;

}


MA1400_R1_IKFastSolver::MA1400_R1_IKFastSolver() {
    init("r1_arm", "clopema_ma1400_kinematics_fast/IKFastKinematicsPlugin", "r1_link_1", "r1_tip_link") ;
}


MA1400_R2_IKSolver::MA1400_R2_IKSolver() {
    init("r2_arm", "clopema_ma1400_kinematics/IKKinematicsPlugin", "r2_link_1", "r2_tip_link") ;
}


MA1400_R2_IKFastSolver::MA1400_R2_IKFastSolver() {
    init("r2_arm", "clopema_ma1400_kinematics_fast/IKFastKinematicsPlugin", "r2_link_1", "r2_tip_link") ;
}


MA1400_R1_Xtion_IKSolver::MA1400_R1_Xtion_IKSolver() {
    init("r1_xtion", "clopema_xtion_kinematics/IKKinematicsPlugin", "r1_link_1", "r1_xtion") ;
}


MA1400_R1_Xtion_IKFastSolver::MA1400_R1_Xtion_IKFastSolver() {
    init("r1_xtion", "clopema_xtion_kinematics_fast/IKFastKinematicsPlugin", "r1_link_1", "r1_xtion") ;
}

MA1400_R2_Xtion_IKSolver::MA1400_R2_Xtion_IKSolver() {
    init("r2_xtion", "clopema_xtion_kinematics/IKKinematicsPlugin", "r2_link_1", "r2_xtion") ;
}

MA1400_R2_Xtion_IKFastSolver::MA1400_R2_Xtion_IKFastSolver() {
    init("r2_xtion", "clopema_xtion_kinematics_fast/IKFastKinematicsPlugin", "r2_link_1", "r2_xtion") ;
}

} // namespace robot_helpers
