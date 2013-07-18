#include <ros/ros.h>
#include <pluginlib/class_loader.h>
#include <kinematics_base/kinematics_base.h>
#include <arm_navigation_msgs/GetPlanningScene.h>
#include <planning_environment/models/model_utils.h>
#include <visualization_msgs/MarkerArray.h>

#include <robot_helpers/Robot.h>
#include <robot_helpers/Utils.h>
#include <robot_helpers/Geometry.h>

using namespace robot_helpers ;
using namespace std ;

class IKSolverInterface {
public:
    IKSolverInterface():  solver_loader("kinematics_base","kinematics::KinematicsBase") {}

    bool init() {
        loadSolver("r1_arm", "clopema_ma1400_kinematics/IKKinematicsPlugin", "r1_arm", "r1_link_1", "r1_tip_link" ) ;
        loadSolver("r2_arm", "clopema_ma1400_kinematics/IKKinematicsPlugin", "r2_arm", "r2_link_1", "r2_tip_link" ) ;
        return true ;
    }

    bool loadSolver(const string &name, const string &instanceName, const string &groupName, const string &baseLink, const string &tipLink)
    {
        SolverTypePtr solver  ;

        try {
            solver = solver_loader.createInstance(instanceName);

            if ( solver && !solver->initialize( groupName, baseLink, tipLink, 0.01) ) {
                ROS_ERROR("Could not initialize kinematics solver for group %s", groupName.c_str());
                return false ;
            }

            solvers[name] = solver ;

        }
        catch(pluginlib::PluginlibException& ex)
        {
            //handle the class failing to load
            ROS_ERROR("The plugin failed to load for some reason. Error: %s", ex.what());
            return false ;
        }

        return true ;

    }

    bool solve(const string &solverName, const Eigen::Vector3d &pos, const Eigen::Quaterniond &orient, const sensor_msgs::JointState &seed_state, sensor_msgs::JointState &state)
    {
        SolverTypePtr solver = solvers[solverName] ;

        geometry_msgs::Pose pose = eigenPoseToROS(pos, orient) ;

        vector<double> solution ;
        int error_code ;

        if ( solver->getPositionIK(pose, seed_state, solution, error_code ) )
        {
            vector<string> joint_names = solver->getJointNames() ;


            for(int i=0 ; i<joint_names.size() ; i++)
            {
                state.name.push_back(joint_names[i]) ;
                state.position.push_back(solution[i]) ;

            }

                return true ;


        }
        else return false ;

    }


    typedef boost::shared_ptr<kinematics::KinematicsBase> SolverTypePtr ;

    pluginlib::ClassLoader<kinematics::KinematicsBase> solver_loader ;
    map<string, SolverTypePtr> solvers ;
};


class PlanningContext {
public:

    PlanningContext() {
    }

    bool init(const std::string &groupName)
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

        return true ;

    }

    void setStateFromJointState(const sensor_msgs::JointState &js)
    {
        map<string, double> joint_state_map ;


        state_->getKinematicStateValues(joint_state_map) ;

        for(int i=0 ; i<js.name.size() ; i++)
            joint_state_map[js.name[i]] = js.position[i] ;


        if ( state_->setKinematicState(joint_state_map) )
            state_->updateKinematicLinks();

    }

    bool isStateValid()
    {
         return ( state_->areJointsWithinBounds(joint_names ) && !cm_->isKinematicStateInCollision(*state_) )  ;
    }

    void getRobotMarkers(visualization_msgs::MarkerArray &markers)
    {

        std_msgs::ColorRGBA rgba ;
        rgba.r = 1 ;
        rgba.g = 0 ;
        rgba.b = 0 ;
        rgba.a = 0.5 ;

        cm_->getRobotMarkersGivenState(*state_, markers, rgba, "robot_markers", ros::Duration(0.1), &link_names);
    }

    void initFromRobotState()
    {
        arm_navigation_msgs::RobotState rs ;

        if ( getRobotState(rs) )
        {
            planning_environment::setRobotStateAndComputeTransforms(rs, *state_);
            cm_->updateAttachedBodyPoses(*state_);

         }
    }

    ~PlanningContext() {
        if (cm_) cm_->revertPlanningScene(state_);
    }

    boost::shared_ptr<planning_environment::CollisionModels> cm_ ;
    planning_models::KinematicState *state_ ;
    vector<string> joint_names, link_names ;
};

int main(int argc, char *argv[])
{


    ros::init(argc, argv, "planning") ;

    ros::NodeHandle nh_ ;

    ros::Publisher pub = nh_.advertise<visualization_msgs::MarkerArray>( "visualization_marker_array", 0 );

 //   IKSolverInterface solvers ;
 //   solvers.init() ;

    PlanningContext ctx ;
    ctx.init("r1_arm") ;

    ctx.initFromRobotState() ;

    sensor_msgs::JointState state ;

    getIK("r1", Eigen::Vector3d(0.2, -0.8, 0.6), lookAt(Eigen::Vector3d(0, 1, 0)), state) ;

    ctx.setStateFromJointState(state) ;

    visualization_msgs::MarkerArray markers ;
    ctx.getRobotMarkers(markers);


    cout << ctx.isStateValid() << endl ;

    while ( ros::ok() )
    {
        pub.publish(markers) ;

        ros::Duration(0.1).sleep() ;
        ros::spinOnce() ;
    }


    cout << "ok" << endl ;
}
