#include <robot_helpers/PlanningContext.h>

#include <ros/ros.h>
#include <pluginlib/class_loader.h>

#include <tf_conversions/tf_eigen.h>

using namespace std ;

class IKSolverLoader {
public:
    IKSolverLoader():  solver_loader("kinematics_base","kinematics::KinematicsBase") {}

    boost::shared_ptr<kinematics::KinematicsBase> load(const string &groupName)
    {
        boost::shared_ptr<kinematics::KinematicsBase> solver  ;

        string instanceName, baseLink, tipLink ;

        ros::param::get("/ompl_planning/" + groupName + "/kinematics_solver", instanceName) ;
        ros::param::get("/ompl_planning/" + groupName + "/root_name", baseLink) ;
        ros::param::get("/ompl_planning/" + groupName + "/tip_name", tipLink) ;

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

namespace robot_helpers {

PlanningContext::PlanningContext() { }

bool PlanningContext::init(const std::string &groupName)
{
	ros::service::waitForService("/environment_server/set_planning_scene_diff");

	arm_navigation_msgs::GetPlanningScene planning_scene;
    
    if (!ros::service::call("/environment_server/set_planning_scene_diff", planning_scene)) {
		ROS_ERROR("Can't get planning scene");
		return false ;
	}

	cm_.reset(new planning_environment::CollisionModels("robot_description")) ;
	state_ = cm_->setPlanningScene(planning_scene.response.planning_scene);

	joint_names = cm_->getKinematicModel()->getModelGroup(groupName)->getJointModelNames();
	link_names = cm_->getKinematicModel()->getModelGroup(groupName)->getUpdatedLinkModelNames();

	IKSolverLoader solvers ;

	solver_ = solvers.load(groupName) ;

	if ( !solver_ ) return false ;

	return true ;
}

void PlanningContext::setStateFromJointState(const sensor_msgs::JointState &js)
{
	map<string, double> joint_state_map ;

    state_->getKinematicStateValues(joint_state_map) ;

    for(int i=0 ; i<js.name.size() ; i++)
		joint_state_map[js.name[i]] = js.position[i] ;

	if ( state_->setKinematicState(joint_state_map) )
		state_->updateKinematicLinks();
}

bool PlanningContext::isStateValid()
{
	return ( state_->areJointsWithinBounds(joint_names ) && !cm_->isKinematicStateInCollision(*state_) )  ;
}

void PlanningContext::getRobotMarkers(visualization_msgs::MarkerArray &markers)
{

	std_msgs::ColorRGBA rgba ;
	rgba.r = 1 ;
	rgba.g = 0 ;
    rgba.b = 0 ;
    rgba.a = 0.5 ;

    cm_->getRobotMarkersGivenState(*state_, markers, rgba, "robot_markers", ros::Duration(0.1), &link_names);
}

void PlanningContext::initFromRobotState()
{
	arm_navigation_msgs::RobotState rs ;

    if ( getRobotState(rs) )
	{
		planning_environment::setRobotStateAndComputeTransforms(rs, *state_);
		cm_->updateAttachedBodyPoses(*state_);
    }
}

Eigen::Affine3d PlanningContext::getTransformToBase(const string &linkName)
{
	const planning_models::KinematicState::LinkState* link_state_ = state_->getLinkState(linkName);
	tf::Transform tr = link_state_->getGlobalLinkTransform() ;

    Eigen::Affine3d etr ;
    tf::TransformTFToEigen(tr, etr) ;
    return etr ;
}


bool PlanningContext::solveIK(const string &tipLinkName, const Eigen::Vector3d &pos, const Eigen::Quaterniond &orient, sensor_msgs::JointState &state)
{
	using namespace Eigen ;

    Affine3d solver_tip_to_pose_tip = getTransformToBase(solver_->getTipName()).inverse() * getTransformToBase(tipLinkName) ;
	Affine3d frame = getTransformToBase(solver_->getBaseName()) ;
    Affine3d pose_ = frame.inverse() *  Translation3d(pos) * orient * solver_tip_to_pose_tip.inverse() ;

    geometry_msgs::Pose pose = eigenPoseToROS(pose_.translation(), Quaterniond(pose_.rotation())) ;

    map<string, double> joint_state_map ;
    state_->getKinematicStateValues(joint_state_map) ;

    vector<double> solution, seed_state_vals ;
    int error_code ;

    vector<string> joint_names = solver_->getJointNames() ;

    for(unsigned int i=0 ; i<joint_names.size() ; i++)
    {
		double val = joint_state_map[joint_names[i]] ;
		seed_state_vals.push_back(val) ;
	}

    if ( solver_->getPositionIK(pose, seed_state_vals, solution, error_code ) )
	{
		for(int i=0 ; i<joint_names.size() ; i++)
		{
			state.name.push_back(joint_names[i]) ;
            state.position.push_back(solution[i]) ;
		}

        return true ;
	}
    else return false ;
}

PlanningContext::~PlanningContext() {
    if (cm_) cm_->revertPlanningScene(state_);
}


} // namespace robot_helpers
