#include "robot_helpers/Utils.h"
#include "robot_helpers/Geometry.h"

#include <geometric_shapes/shape_operations.h>
#include <planning_environment/util/construct_object.h>
#include <geometry_msgs/Pose.h>
#include <arm_navigation_msgs/CollisionObject.h>
#include <arm_navigation_msgs/GetPlanningScene.h>

#include <visualization_msgs/MarkerArray.h>
#include <planning_environment/models/collision_models.h>
#include <kinematics_msgs/GetConstraintAwarePositionIK.h>


using namespace std ;

namespace robot_helpers {

bool moveHome(MoveRobot &cmove)
{

    clopema_arm_navigation::ClopemaMotionPlan mp;
    mp.request.motion_plan_req.group_name = "arms";
    mp.request.motion_plan_req.allowed_planning_time = ros::Duration(5.0);

    if (!getRobotState(mp.request.motion_plan_req.start_state)) return false ;

    vector<string> joint_names ;

    joint_names.push_back("r1_joint_s");
    joint_names.push_back("r1_joint_l");
    joint_names.push_back("r1_joint_u");
    joint_names.push_back("r1_joint_r");
    joint_names.push_back("r1_joint_b");
    joint_names.push_back("r1_joint_t");

    joint_names.push_back("r2_joint_s");
    joint_names.push_back("r2_joint_l");
    joint_names.push_back("r2_joint_u");
    joint_names.push_back("r2_joint_r");
    joint_names.push_back("r2_joint_b");
    joint_names.push_back("r2_joint_t");

    mp.request.motion_plan_req.goal_constraints.joint_constraints.resize(joint_names.size());

    for (unsigned int i = 0; i < mp.request.motion_plan_req.goal_constraints.joint_constraints.size(); ++i) {
        mp.request.motion_plan_req.goal_constraints.joint_constraints[i].joint_name = joint_names[i];
        mp.request.motion_plan_req.goal_constraints.joint_constraints[i].position = 0.0;
        mp.request.motion_plan_req.goal_constraints.joint_constraints[i].tolerance_below = 0.1;
        mp.request.motion_plan_req.goal_constraints.joint_constraints[i].tolerance_above = 0.1;
    }

    if (!plan(mp)) return false ;

    control_msgs::FollowJointTrajectoryGoal goal;
    goal.trajectory = mp.response.joint_trajectory;
    cmove.doGoal(goal);

    mp.request.motion_plan_req.group_name = "ext_axis";
    mp.request.motion_plan_req.allowed_planning_time = ros::Duration(5.0);
    if (!getRobotState(mp.request.motion_plan_req.start_state)) return false ;

    joint_names.clear() ;
    joint_names.push_back("ext_axis") ;

    mp.request.motion_plan_req.goal_constraints.joint_constraints.resize(joint_names.size());
    for (unsigned int i = 0; i < mp.request.motion_plan_req.goal_constraints.joint_constraints.size(); ++i) {
        mp.request.motion_plan_req.goal_constraints.joint_constraints[i].joint_name = joint_names[i];
        mp.request.motion_plan_req.goal_constraints.joint_constraints[i].position = 0.0;
        mp.request.motion_plan_req.goal_constraints.joint_constraints[i].tolerance_below = 0.1;
        mp.request.motion_plan_req.goal_constraints.joint_constraints[i].tolerance_above = 0.1;
    }

    if ( !plan(mp) ) return false ;


    goal.trajectory = mp.response.joint_trajectory;
    cmove.doGoal(goal);

    return true ;
}





bool moveGripper(MoveRobot &cmove, const std::string &armName, const Eigen::Vector3d &pos, const Eigen::Quaterniond &q)
{
    trajectory_msgs::JointTrajectory traj ;

    return ( planArmToPose(armName, pos, q, traj) && cmove.execTrajectory(traj) ) ;
}

bool planToJointGoal(const string &armName, const sensor_msgs::JointState &js, trajectory_msgs::JointTrajectory &traj)
{

    clopema_arm_navigation::ClopemaMotionPlan mp;
    mp.request.motion_plan_req.group_name = armName + "_arm";
    mp.request.motion_plan_req.allowed_planning_time = ros::Duration(5.0);

    if (!getRobotState(mp.request.motion_plan_req.start_state)) return false ;

    vector<string> joint_names ;

    joint_names.push_back(armName + "_joint_s");
    joint_names.push_back(armName + "_joint_l");
    joint_names.push_back(armName + "_joint_u");
    joint_names.push_back(armName + "_joint_r");
    joint_names.push_back(armName + "_joint_b");
    joint_names.push_back(armName + "_joint_t");

    mp.request.motion_plan_req.goal_constraints.joint_constraints.resize(joint_names.size());

    for (unsigned int i = 0; i < mp.request.motion_plan_req.goal_constraints.joint_constraints.size(); ++i)
    {
        mp.request.motion_plan_req.goal_constraints.joint_constraints[i].joint_name = joint_names[i];
        mp.request.motion_plan_req.goal_constraints.joint_constraints[i].tolerance_below = 0.1;
        mp.request.motion_plan_req.goal_constraints.joint_constraints[i].tolerance_above = 0.1;

        vector<string>::const_iterator it = std::find(js.name.begin(), js.name.end(), joint_names[i]) ;

        if ( it == js.name.end() )
            mp.request.motion_plan_req.goal_constraints.joint_constraints[i].position = 0;
        else {
            int d = std::distance(js.name.begin(), it) ;
            mp.request.motion_plan_req.goal_constraints.joint_constraints[i].position = js.position[d];
        }
    }

    if (!plan(mp)) return false ;

    traj = mp.response.joint_trajectory ;

    if ( traj.points.size() > 2 ) return true ;

}

bool planArmToPose(const string &armName, const Eigen::Vector3d &pos, const Eigen::Quaterniond &q, trajectory_msgs::JointTrajectory &traj)
{
    clopema_arm_navigation::ClopemaMotionPlan mp;
    mp.request.motion_plan_req.group_name = armName + "_arm" ;
    mp.request.motion_plan_req.allowed_planning_time = ros::Duration(5.0);

    if ( !getRobotState(mp.request.motion_plan_req.start_state) ) return false ;

    arm_navigation_msgs::SimplePoseConstraint desired_pose;

    desired_pose.header.frame_id = "base_link";
    desired_pose.header.stamp = ros::Time::now();
    desired_pose.link_name = armName + "_ee";
    desired_pose.pose.position.x = pos.x() ;
    desired_pose.pose.position.y = pos.y() ;
    desired_pose.pose.position.z = pos.z() ;

    desired_pose.pose.orientation.x = q.x() ;
    desired_pose.pose.orientation.y = q.y() ;
    desired_pose.pose.orientation.z = q.z() ;
    desired_pose.pose.orientation.w = q.w() ;

    desired_pose.absolute_position_tolerance.x = desired_pose.absolute_position_tolerance.y = desired_pose.absolute_position_tolerance.z = 0.02;
    desired_pose.absolute_roll_tolerance = desired_pose.absolute_pitch_tolerance = desired_pose.absolute_yaw_tolerance = 0.04;

    poseToClopemaMotionPlan(mp, desired_pose);

    if (!plan(mp)) return false;

    traj = mp.response.joint_trajectory ;

    if ( traj.points.size() > 2 ) return true ;

}


bool moveGripperPointingDown(MoveRobot &cmove, const std::string &armName, double X, double Y, double Z)
{
    Eigen::Quaterniond q ;
    q = Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()) ;
    return moveGripper(cmove, armName, Eigen::Vector3d(X, Y, Z), q) ;
}

bool rotateGripper(MoveRobot &cmove, const std::string &armName, double theta)
{
    clopema_arm_navigation::ClopemaMotionPlan mp;

    mp.request.motion_plan_req.group_name = armName + "_arm";
    mp.request.motion_plan_req.allowed_planning_time = ros::Duration(5.0);

    if ( !getRobotState(mp.request.motion_plan_req.start_state) ) return false ;

    std::vector<std::string> names;
    names.push_back(armName + "_joint_s");
    names.push_back(armName + "_joint_l");
    names.push_back(armName + "_joint_u");
    names.push_back(armName + "_joint_r");
    names.push_back(armName + "_joint_b");
    names.push_back(armName + "_joint_t");

    string gripperJointName = armName + "_joint_t" ;

    mp.request.motion_plan_req.goal_constraints.joint_constraints.resize(names.size());

    for( int i=0 ; i<names.size() ; i++ )
    {
        mp.request.motion_plan_req.goal_constraints.joint_constraints[i].joint_name = names[i];
        mp.request.motion_plan_req.goal_constraints.joint_constraints[i].tolerance_below = 0.1;
        mp.request.motion_plan_req.goal_constraints.joint_constraints[i].tolerance_above = 0.1;

        for(int j=0 ; j< mp.request.motion_plan_req.start_state.joint_state.name.size() ; j++ )
        {
            if ( mp.request.motion_plan_req.start_state.joint_state.name[j] == names[i] )
            {
                if ( names[i] == gripperJointName )
                    mp.request.motion_plan_req.goal_constraints.joint_constraints[i].position =
                            mp.request.motion_plan_req.start_state.joint_state.position[j] + theta ;
                else
                    mp.request.motion_plan_req.goal_constraints.joint_constraints[i].position =
                            mp.request.motion_plan_req.start_state.joint_state.position[j]  ;
            }
        }

    }

    if (!plan(mp)) return false ;

    control_msgs::FollowJointTrajectoryGoal goal;
    goal.trajectory = mp.response.joint_trajectory;
    cmove.doGoal(goal);

    return true ;
}


bool planXtionToPose(const string &armName, const Eigen::Vector3d &pos, const Eigen::Quaterniond &q, trajectory_msgs::JointTrajectory &traj)
{
    sensor_msgs::JointState goal_state ;

    if ( !getIKXtion(armName, pos, q, goal_state) ) return false ;

    return planToJointGoal(armName, goal_state, traj) ;
}


////////////////////////////////////////////////////////////////////////////////////////////////////////

// We add a collision object as attached object to the gripper. the parent frame is set to the baselink since otherwise the
// cone will rotate as the gripper moves. Thus this function may be used for one arm static or by calling resetCollisionModel, moving the arm,
// and then setting again the collision object. This is a peculiarity of non-rigid attachement.

bool addConeToCollisionModel(const std::string &armName, double length, double radius)
{
    ros::NodeHandle nh("~") ;

    ros::service::waitForService("/environment_server/set_planning_scene_diff");
    ros::ServiceClient get_planning_scene_client =
      nh.serviceClient<arm_navigation_msgs::GetPlanningScene>("/environment_server/set_planning_scene_diff");

    arm_navigation_msgs::GetPlanningScene::Request planning_scene_req;
    arm_navigation_msgs::GetPlanningScene::Response planning_scene_res;

    arm_navigation_msgs::AttachedCollisionObject att_object;
    att_object.link_name = armName + "_gripper";

    att_object.object.id = "attached_cone";
    att_object.object.operation.operation = arm_navigation_msgs::CollisionObjectOperation::ADD;

    att_object.object.header.frame_id = "base_link";
    att_object.object.header.stamp = ros::Time::now();

    Eigen::Vector3d p = robot_helpers::getPose(armName).inverse().translation() ;


    arm_navigation_msgs::Shape object;

    shapes::Mesh mesh ;
    makeSolidCone(mesh, radius, length, 10, 20) ;

    if(!planning_environment::constructObjectMsg(&mesh, object)) {
      ROS_WARN_STREAM("Object construction fails");
    }

    geometry_msgs::Pose pose;
    pose.position.x = p.x();
    pose.position.y = p.y();
    pose.position.z = p.z() ;
    pose.orientation.x = 0;
    pose.orientation.y = 0;
    pose.orientation.z = 0;
    pose.orientation.w = 1;

    att_object.object.shapes.push_back(object);
    att_object.object.poses.push_back(pose);

    planning_scene_req.planning_scene_diff.attached_collision_objects.push_back(att_object);

    if(!get_planning_scene_client.call(planning_scene_req, planning_scene_res)) return false;


    return true ;
}


bool addSphereToCollisionModel(const std::string &armName, double radius)
{
    std::string arm2Name;
    ros::NodeHandle nh("~") ;

    ros::service::waitForService("/environment_server/set_planning_scene_diff");
    ros::ServiceClient get_planning_scene_client =
      nh.serviceClient<arm_navigation_msgs::GetPlanningScene>("/environment_server/set_planning_scene_diff");

    arm_navigation_msgs::GetPlanningScene::Request planning_scene_req;
    arm_navigation_msgs::GetPlanningScene::Response planning_scene_res;

    arm_navigation_msgs::AttachedCollisionObject att_object;

    att_object.link_name = armName + "_gripper";

    att_object.touch_links.push_back(armName +"_link_1");
    att_object.touch_links.push_back(armName +"_link_2");
    att_object.touch_links.push_back(armName +"_link_3");
    att_object.touch_links.push_back(armName +"_link_4");
    att_object.touch_links.push_back(armName +"_link_5"); 
    att_object.touch_links.push_back(armName +"_link_6");
    att_object.touch_links.push_back(armName +"_tip_link");
    att_object.touch_links.push_back(armName +"_ee");
    att_object.touch_links.push_back(armName +"_xtion");
    att_object.touch_links.push_back(armName +"_cable_1");
    att_object.touch_links.push_back("r750_base");
    att_object.touch_links.push_back("base_link");
    
    if (armName=="r1")
       arm2Name="r2";
    else
       arm2Name="r1";

    att_object.touch_links.push_back(arm2Name + "_link_1");
    att_object.touch_links.push_back(arm2Name + "_link_2");
    att_object.touch_links.push_back(arm2Name + "_link_3");
    att_object.touch_links.push_back(arm2Name + "_link_4");
    att_object.touch_links.push_back(arm2Name + "_link_5"); 
    att_object.touch_links.push_back(arm2Name + "_link_6");
    att_object.touch_links.push_back(arm2Name + "_tip_link");
    att_object.touch_links.push_back(arm2Name + "_xtion");
    att_object.touch_links.push_back(arm2Name + "_cable_1");



    att_object.object.id = "/attached_sphere";
    att_object.object.operation.operation = arm_navigation_msgs::CollisionObjectOperation::ADD;

    att_object.object.header.frame_id = "base_link";
    att_object.object.header.stamp = ros::Time::now();


	
    Eigen::Vector3d p = robot_helpers::getPose(armName).inverse().translation() ;


    arm_navigation_msgs::Shape object;

    object.type = arm_navigation_msgs::Shape::SPHERE;
    object.dimensions.resize(1);
    object.dimensions[0] = radius;

    geometry_msgs::Pose pose;
    pose.position.x = p.x();
    pose.position.y = p.y();
    pose.position.z = p.z();

    pose.orientation.x = 0;
    pose.orientation.y = 0;
    pose.orientation.z = 0;
    pose.orientation.w = 1;

    att_object.object.shapes.push_back(object);
    att_object.object.poses.push_back(pose);

    planning_scene_req.planning_scene_diff.attached_collision_objects.push_back(att_object);

    if(!get_planning_scene_client.call(planning_scene_req, planning_scene_res)) return false;


    return true ;
}


bool resetCollisionModel()
{
    ros::NodeHandle nh("~") ;

    ros::service::waitForService("/environment_server/set_planning_scene_diff");
    ros::ServiceClient set_planning_scene_client =
      nh.serviceClient<arm_navigation_msgs::GetPlanningScene>("/environment_server/set_planning_scene_diff");

    arm_navigation_msgs::GetPlanningScene::Request planning_scene_req;
    arm_navigation_msgs::GetPlanningScene::Response planning_scene_res;

    arm_navigation_msgs::AttachedCollisionObject att_object;

    att_object.object.id = "all";
    att_object.object.operation.operation = arm_navigation_msgs::CollisionObjectOperation::REMOVE;

    att_object.object.header.frame_id = "base_link";
    att_object.object.header.stamp = ros::Time::now();

    planning_scene_req.planning_scene_diff.attached_collision_objects.push_back(att_object);

    if(!set_planning_scene_client.call(planning_scene_req, planning_scene_res))
       return false;


}

using namespace Eigen ;

geometry_msgs::Pose eigenPoseToROS(const Vector3d &pos, const Quaterniond &orient)
{
    geometry_msgs::Pose pose ;

    pose.position.x = pos.x() ;
    pose.position.y = pos.y() ;
    pose.position.z = pos.z() ;

    pose.orientation.x = orient.x() ;
    pose.orientation.y = orient.y() ;
    pose.orientation.z = orient.z() ;
    pose.orientation.w = orient.w() ;

    return pose ;

}

bool getIK(const string &armName, const Eigen::Vector3d pos, const Eigen::Quaterniond &q, sensor_msgs::JointState &state) {

    geometry_msgs::Pose pose = eigenPoseToROS(pos, q);

    kinematics_msgs::GetConstraintAwarePositionIK ik;
    ik.request.timeout = ros::Duration(5.0);
    ik.request.ik_request.ik_link_name = armName + "_ee";
    ik.request.ik_request.pose_stamped.header.frame_id = "base_link";
    ik.request.ik_request.pose_stamped.header.stamp = ros::Time::now();


    if (!getRobotState(ik.request.ik_request.robot_state)) {
        ROS_ERROR("Can't get robot state");
        return false ;
    }

    ik.request.ik_request.ik_seed_state = ik.request.ik_request.robot_state;
    ik.request.ik_request.pose_stamped.pose = pose;

    string serviceName = "/clopema_" + armName + "_arm_kinematics/get_constraint_aware_ik" ;

    ros::service::waitForService(serviceName) ;

    if (ros::service::call(serviceName, ik)) {
        if (ik.response.error_code.val == ik.response.error_code.SUCCESS)
            ROS_DEBUG_STREAM("IK Solution found");
        else {
            ROS_WARN_STREAM("Error code: " << ik.response.error_code.val<<": "<<arm_navigation_msgs::armNavigationErrorCodeToString(ik.response.error_code));
            return false ;
        }
    } else {
        ROS_ERROR("Can't call service");
        return false ;
    }

    state = ik.response.solution.joint_state ;
    return true ;
}

bool getIKXtion(const string &armName, const Eigen::Vector3d pos, const Eigen::Quaterniond &q, sensor_msgs::JointState &state) {

    geometry_msgs::Pose pose = eigenPoseToROS(pos, q);


    kinematics_msgs::GetConstraintAwarePositionIK ik;
    ik.request.timeout = ros::Duration(5.0);
    ik.request.ik_request.ik_link_name = armName + "_xtion";
    ik.request.ik_request.pose_stamped.header.frame_id = "base_link";
    ik.request.ik_request.pose_stamped.header.stamp = ros::Time::now();
    ik.request.ik_request.pose_stamped.pose = pose;

    if (!getRobotState(ik.request.ik_request.robot_state)) {
        ROS_ERROR("Can't get robot state");
        return false ;
    }

    ik.request.ik_request.ik_seed_state = ik.request.ik_request.robot_state;

    string serviceName = "/clopema_" + armName + "_xtion_kinematics/get_constraint_aware_ik" ;

    ros::service::waitForService(serviceName) ;

    if (ros::service::call(serviceName, ik)) {
        if (ik.response.error_code.val == ik.response.error_code.SUCCESS)
            ROS_DEBUG_STREAM("IK Solution found");
        else {
            ROS_WARN_STREAM("Error code: " << ik.response.error_code.val<<": "<<arm_navigation_msgs::armNavigationErrorCodeToString(ik.response.error_code));
            return false ;
        }
    } else {
        ROS_ERROR("Can't call service");
        return false ;
    }

    state = ik.response.solution.joint_state ;
    return true ;
}









}
