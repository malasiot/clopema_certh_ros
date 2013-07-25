#include "robot_helpers/Utils.h"
#include "robot_helpers/Unfold.h"
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

    bool plan_result = planArmToPose(armName, pos, q, traj) ;
    bool exec_traj = cmove.execTrajectory(traj) ;
    return ( plan_result && exec_traj ) ;
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

    return false;

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

    return false;
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

    for( unsigned int i=0 ; i<names.size() ; i++ )
    {
        mp.request.motion_plan_req.goal_constraints.joint_constraints[i].joint_name = names[i];
        mp.request.motion_plan_req.goal_constraints.joint_constraints[i].tolerance_below = 0.1;
        mp.request.motion_plan_req.goal_constraints.joint_constraints[i].tolerance_above = 0.1;

        for( unsigned int j=0 ; j< mp.request.motion_plan_req.start_state.joint_state.name.size() ; j++ )
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

    return true;

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



/////////// andreas



int moveArmConstrains(geometry_msgs::Pose pose, const string &armName, float radious){
        //Create plan
        clopema_arm_navigation::ClopemaMotionPlan mp;
        MoveRobot cmove;
        cmove.setServoMode(false);
        mp.request.motion_plan_req.group_name = armName + "_arm";
        mp.request.motion_plan_req.allowed_planning_time = ros::Duration(5.0);

        //Set start state
        getRobotState(mp.request.motion_plan_req.start_state);

        arm_navigation_msgs::SimplePoseConstraint desired_pose;

        desired_pose.header.frame_id = "base_link";
        desired_pose.header.stamp = ros::Time::now();
        desired_pose.link_name = armName + "_ee";

        desired_pose.pose = pose;

       // cout<< "\n going to --- >  "<< desired_pose.pose.position.x<< " "  << desired_pose.pose.position.y<<"  " << desired_pose.pose.position.z <<"\n";

        desired_pose.absolute_position_tolerance.x = 0.002;
        desired_pose.absolute_position_tolerance.y = 0.002;
        desired_pose.absolute_position_tolerance.z = 0.002;
        desired_pose.absolute_roll_tolerance = 0.004;
        desired_pose.absolute_pitch_tolerance = 0.004;
        desired_pose.absolute_yaw_tolerance = 0.004;

        geometry_msgs::Quaternion q;
        q.x=0;
        q.y=0;
        q.z=0;
        q.w=1;

        setPathConstraints(mp, radious, armName, q);

        poseToClopemaMotionPlan(mp, desired_pose);

        ROS_INFO("Planning");
        if (!plan(mp))
            return -1;

        ROS_INFO("Executing");
        control_msgs::FollowJointTrajectoryGoal goal;
        goal.trajectory = mp.response.joint_trajectory;
        cmove.doGoal(goal);
        return 0;
}






void setPathConstraints(clopema_arm_navigation::ClopemaMotionPlan & mp, float radious , const string &armName, geometry_msgs::Quaternion q ) {

    string arm2Name="r1";
    if (armName == "r1")
        arm2Name="r2";

    mp.request.motion_plan_req.path_constraints.position_constraints.resize(1);
    mp.request.motion_plan_req.path_constraints.position_constraints[0].header.frame_id = armName + "_ee";
    mp.request.motion_plan_req.path_constraints.position_constraints[0].header.stamp = ros::Time::now();
    mp.request.motion_plan_req.path_constraints.position_constraints[0].link_name = arm2Name + "_ee";
    mp.request.motion_plan_req.path_constraints.position_constraints[0].position.x = 0.0;
    mp.request.motion_plan_req.path_constraints.position_constraints[0].position.y = 0.0;
    mp.request.motion_plan_req.path_constraints.position_constraints[0].position.z = 0.0;
    mp.request.motion_plan_req.path_constraints.position_constraints[0].constraint_region_orientation.x = 1;
    mp.request.motion_plan_req.path_constraints.position_constraints[0].constraint_region_orientation.y = 0;
    mp.request.motion_plan_req.path_constraints.position_constraints[0].constraint_region_orientation.z = 0;
    mp.request.motion_plan_req.path_constraints.position_constraints[0].constraint_region_orientation.w = 0;

    mp.request.motion_plan_req.path_constraints.position_constraints[0].constraint_region_shape.type = arm_navigation_msgs::Shape::SPHERE;
    mp.request.motion_plan_req.path_constraints.position_constraints[0].constraint_region_shape.dimensions.push_back(radious); //radius
    mp.request.motion_plan_req.path_constraints.position_constraints[0].constraint_region_shape.dimensions.push_back(radious);
    mp.request.motion_plan_req.path_constraints.position_constraints[0].constraint_region_shape.dimensions.push_back(radious);
    mp.request.motion_plan_req.path_constraints.position_constraints[0].weight = 1.0;

//    mp.request.motion_plan_req.path_constraints.orientation_constraints.resize(1);
//    mp.request.motion_plan_req.path_constraints.orientation_constraints[0].header.frame_id = "base_link";
//    mp.request.motion_plan_req.path_constraints.orientation_constraints[0].header.stamp = ros::Time::now();
//    mp.request.motion_plan_req.path_constraints.orientation_constraints[0].link_name = armName+ "_ee";

//    geometry_msgs::Pose p= getArmPose(armName);

//    mp.request.motion_plan_req.path_constraints.orientation_constraints[0].orientation=p.orientation;
//    mp.request.motion_plan_req.path_constraints.orientation_constraints[0].type = arm_navigation_msgs::OrientationConstraint::HEADER_FRAME;
//    mp.request.motion_plan_req.path_constraints.orientation_constraints[0].absolute_roll_tolerance = 3.14;
//    mp.request.motion_plan_req.path_constraints.orientation_constraints[0].absolute_pitch_tolerance = 3.14;
//    mp.request.motion_plan_req.path_constraints.orientation_constraints[0].absolute_yaw_tolerance = 3.14;
//    mp.request.motion_plan_req.path_constraints.orientation_constraints[0].weight = 1.0;

}

void rotateGripper(float angle ,const string &armName){

    //Create plan
    clopema_arm_navigation::ClopemaMotionPlan mp;
    MoveRobot cmove;
    cmove.setServoMode(false);

    mp.request.motion_plan_req.group_name = armName + "_arm";
    mp.request.motion_plan_req.allowed_planning_time = ros::Duration(5.0);

    //Set start state
    getRobotState(mp.request.motion_plan_req.start_state);

    arm_navigation_msgs::SimplePoseConstraint desired_pose;

    desired_pose.header.frame_id = armName + "_ee";
    desired_pose.header.stamp = ros::Time::now();
    desired_pose.link_name = armName + "_ee";

    desired_pose.pose.position.x = 0;
    desired_pose.pose.position.y = 0;
    desired_pose.pose.position.z = 0;
    desired_pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, angle );


   // cout<< "\n going to --- >  "<< desired_pose.pose.position.x<< " "  << desired_pose.pose.position.y<<"  " << desired_pose.pose.position.z <<"\n";

    desired_pose.absolute_position_tolerance.x = 0.002;
    desired_pose.absolute_position_tolerance.y = 0.002;
    desired_pose.absolute_position_tolerance.z = 0.002;
    desired_pose.absolute_roll_tolerance = 0.004;
    desired_pose.absolute_pitch_tolerance = 0.004;
    desired_pose.absolute_yaw_tolerance = 0.004;
    poseToClopemaMotionPlan(mp, desired_pose);

    ROS_INFO("Planning");
    if (!plan(mp))
        return ;

    ROS_INFO("Executing");
    control_msgs::FollowJointTrajectoryGoal goal;
    goal.trajectory = mp.response.joint_trajectory;
    cmove.doGoal(goal);

}

int moveArmBetweenSpheres( string armName, bool up, geometry_msgs::Pose goalPose){

    string otherArm= "r1";
    if (armName == "r1")
        otherArm ="r2";

    float radious = getArmsDistance();

    tf::StampedTransform st = getTranformation(otherArm + "_ee");
    geometry_msgs::Point goalPoint;
    goalPoint.x = st.getOrigin().x();
    goalPoint.y = st.getOrigin().y();
    goalPoint.z = st.getOrigin().z();

    if(armName == "r2"){
        if (up == true){
            goalPoint.x += radious;
        }
        else{
            goalPoint.z -= radious;
        }
    }
    else{
        if (up==true){
            goalPoint.x -= radious;
        }
        else{
            goalPoint.z -= radious;
        }
    }

    goalPose.position=goalPoint;
    addSphereToCollisionModel(otherArm, radious/2.0);

    if ( moveArmConstrains( goalPose, armName, radious + 0.1) == -1){
        cout<<"ABORDING..." <<endl;
        return -1;
    }

    resetCollisionModel();

    return 0;
}


int moveArm(geometry_msgs::Pose pose, const string &armName){

        //Create plan
        clopema_arm_navigation::ClopemaMotionPlan mp;
        MoveRobot cmove;

        cmove.setServoMode(false);
        mp.request.motion_plan_req.group_name = armName + "_arm";
        mp.request.motion_plan_req.allowed_planning_time = ros::Duration(5.0);

        //Set start state
        getRobotState(mp.request.motion_plan_req.start_state);

        arm_navigation_msgs::SimplePoseConstraint desired_pose;

        desired_pose.header.frame_id = "base_link";
        desired_pose.header.stamp = ros::Time::now();
        desired_pose.link_name = armName + "_ee";
        desired_pose.pose = pose;
        desired_pose.absolute_position_tolerance.x = 0.002;
        desired_pose.absolute_position_tolerance.y = 0.002;
        desired_pose.absolute_position_tolerance.z = 0.002;
        desired_pose.absolute_roll_tolerance = 0.004;
        desired_pose.absolute_pitch_tolerance = 0.004;
        desired_pose.absolute_yaw_tolerance = 0.004;
        poseToClopemaMotionPlan(mp, desired_pose);

        ROS_INFO("Planning");
        if (!plan(mp))
            return -1;

        ROS_INFO("Executing");
        control_msgs::FollowJointTrajectoryGoal goal;
        goal.trajectory = mp.response.joint_trajectory;
        cmove.doGoal(goal);

        return 0;
}


int moveArms( geometry_msgs::Pose pose1, geometry_msgs::Pose pose2, const string &arm1Name, const string &arm2Name){

    MoveRobot cmove;
    cmove.setServoMode(false);

     //Create plan
    clopema_arm_navigation::ClopemaMotionPlan mp;
    mp.request.motion_plan_req.group_name = "arms";
    mp.request.motion_plan_req.allowed_planning_time = ros::Duration(5.0);

    //Set start state
    if (!getRobotState(mp.request.motion_plan_req.start_state))
        return -1;

    arm_navigation_msgs::SimplePoseConstraint desired_pose;
    desired_pose.header.frame_id = "base_link";
    desired_pose.header.stamp = ros::Time::now();
    desired_pose.link_name = arm2Name + "_ee";
    desired_pose.pose=pose2;

    desired_pose.absolute_position_tolerance.x = 0.02;
    desired_pose.absolute_position_tolerance.y = 0.02;
    desired_pose.absolute_position_tolerance.z = 0.02;
    desired_pose.absolute_roll_tolerance = 0.04;
    desired_pose.absolute_pitch_tolerance = 0.04;
    desired_pose.absolute_yaw_tolerance = 0.04;


    arm_navigation_msgs::PositionConstraint position_constraint;
    arm_navigation_msgs::OrientationConstraint orientation_constraint;
    arm_navigation_msgs::poseConstraintToPositionOrientationConstraints(desired_pose, position_constraint, orientation_constraint);
    mp.request.motion_plan_req.goal_constraints.orientation_constraints.push_back(orientation_constraint);
    mp.request.motion_plan_req.goal_constraints.position_constraints.push_back(position_constraint);

    //add r1 goal constraints
    desired_pose.link_name = arm1Name + "_ee";
    desired_pose.pose=pose1;

    arm_navigation_msgs::poseConstraintToPositionOrientationConstraints(desired_pose, position_constraint, orientation_constraint);
    mp.request.motion_plan_req.goal_constraints.orientation_constraints.push_back(orientation_constraint);
    mp.request.motion_plan_req.goal_constraints.position_constraints.push_back(position_constraint);

    ROS_INFO("Planning");
    if (!plan(mp))
        return -1;

    ROS_INFO("Executing");
    control_msgs::FollowJointTrajectoryGoal goal;
    goal.trajectory = mp.response.joint_trajectory;
    cmove.doGoal(goal);

    return 0;
}

int moveArmsNoTearing( geometry_msgs::Pose pose1, geometry_msgs::Pose pose2, const string &arm1Name, const string &arm2Name){

    float radious = getArmsDistance()+0.1;

    MoveRobot cmove;
    cmove.setServoMode(false);
     //Create plan
    clopema_arm_navigation::ClopemaMotionPlan mp;
    mp.request.motion_plan_req.group_name = "arms";
    mp.request.motion_plan_req.allowed_planning_time = ros::Duration(5.0);

    //Set start state
    if (!getRobotState(mp.request.motion_plan_req.start_state))
        return -1;

    arm_navigation_msgs::SimplePoseConstraint desired_pose;
    desired_pose.header.frame_id = "base_link";
    desired_pose.header.stamp = ros::Time::now();
    desired_pose.link_name = arm2Name + "_ee";
    desired_pose.pose=pose2;

    desired_pose.absolute_position_tolerance.x = 0.02;
    desired_pose.absolute_position_tolerance.y = 0.02;
    desired_pose.absolute_position_tolerance.z = 0.02;
    desired_pose.absolute_roll_tolerance = 0.04;
    desired_pose.absolute_pitch_tolerance = 0.04;
    desired_pose.absolute_yaw_tolerance = 0.04;


    mp.request.motion_plan_req.path_constraints.position_constraints.resize(2);
    mp.request.motion_plan_req.path_constraints.position_constraints[0].header.frame_id = arm1Name + "_ee";
    mp.request.motion_plan_req.path_constraints.position_constraints[0].header.stamp = ros::Time::now();
    mp.request.motion_plan_req.path_constraints.position_constraints[0].link_name = arm2Name + "_ee";
    mp.request.motion_plan_req.path_constraints.position_constraints[0].position.x = 0;
    mp.request.motion_plan_req.path_constraints.position_constraints[0].position.y = 0;
    mp.request.motion_plan_req.path_constraints.position_constraints[0].position.z = 0;
    mp.request.motion_plan_req.path_constraints.position_constraints[0].constraint_region_shape.type = arm_navigation_msgs::Shape::SPHERE;
    mp.request.motion_plan_req.path_constraints.position_constraints[0].constraint_region_shape.dimensions.push_back(radious); //radius
    mp.request.motion_plan_req.path_constraints.position_constraints[0].constraint_region_shape.dimensions.push_back(radious);
    mp.request.motion_plan_req.path_constraints.position_constraints[0].constraint_region_shape.dimensions.push_back(radious);
    mp.request.motion_plan_req.path_constraints.position_constraints[0].constraint_region_orientation.x = 0.0;
    mp.request.motion_plan_req.path_constraints.position_constraints[0].constraint_region_orientation.y = 0.0;
    mp.request.motion_plan_req.path_constraints.position_constraints[0].constraint_region_orientation.z = 0.0;
    mp.request.motion_plan_req.path_constraints.position_constraints[0].constraint_region_orientation.w = 1.0;
    mp.request.motion_plan_req.path_constraints.position_constraints[0].weight = 1.0;
    mp.request.motion_plan_req.path_constraints.position_constraints[1] = mp.request.motion_plan_req.path_constraints.position_constraints[0];
    mp.request.motion_plan_req.path_constraints.position_constraints[1].link_name = arm1Name + "_ee";
    mp.request.motion_plan_req.path_constraints.position_constraints[1].position.x = 0.4;

    arm_navigation_msgs::PositionConstraint position_constraint;
    arm_navigation_msgs::OrientationConstraint orientation_constraint;
    arm_navigation_msgs::poseConstraintToPositionOrientationConstraints(desired_pose, position_constraint, orientation_constraint);
    mp.request.motion_plan_req.goal_constraints.orientation_constraints.push_back(orientation_constraint);
    mp.request.motion_plan_req.goal_constraints.position_constraints.push_back(position_constraint);

    //add r1 goal constraints
    desired_pose.link_name = arm1Name + "_ee";
    desired_pose.pose=pose1;
    arm_navigation_msgs::poseConstraintToPositionOrientationConstraints(desired_pose, position_constraint, orientation_constraint);
    mp.request.motion_plan_req.goal_constraints.orientation_constraints.push_back(orientation_constraint);
    mp.request.motion_plan_req.goal_constraints.position_constraints.push_back(position_constraint);

    ROS_INFO("Planning");
    if (!plan(mp))
        return -1;

    ROS_INFO("Executing");
    control_msgs::FollowJointTrajectoryGoal goal;
    goal.trajectory = mp.response.joint_trajectory;
    cmove.doGoal(goal);

    return 0;
}



bool moveHomeArm(const string &armName){

    MoveRobot cmove;
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


int moveArmThrough(vector <geometry_msgs::Pose> poses , const string &armName){


    MoveRobot cmove;
    cmove.setServoMode(false);
    trajectory_msgs::JointTrajectory wholeTraj;

    //Create plan
    clopema_arm_navigation::ClopemaMotionPlan mp;
    mp.request.motion_plan_req.group_name = armName + "_arm";
    mp.request.motion_plan_req.allowed_planning_time = ros::Duration(5.0);

    if (!getRobotState(mp.request.motion_plan_req.start_state))
        return -1;

    vector <arm_navigation_msgs::SimplePoseConstraint> desired_poses;
    arm_navigation_msgs::SimplePoseConstraint desPose;

    for (unsigned int i = 0; i < poses.size() ; i ++ ){

        desPose.header.frame_id = "base_link";
        desPose.header.stamp = ros::Time::now();
        desPose.link_name = armName + "_ee";
        desPose.pose = poses[i];
        desPose.absolute_position_tolerance.x = 0.02;
        desPose.absolute_position_tolerance.y = 0.02;
        desPose.absolute_position_tolerance.z = 0.02;
        desPose.absolute_roll_tolerance = 0.04;
        desPose.absolute_pitch_tolerance = 0.04;
        desPose.absolute_yaw_tolerance = 0.04;
        desired_poses.push_back(desPose);

    }

    poseToClopemaMotionPlan(mp, desired_poses[0]);
    if (!plan(mp))
        return -1;
    wholeTraj = mp.response.joint_trajectory;

    for(unsigned int i=1;i<desired_poses.size();i++){
        mp.request.motion_plan_req.goal_constraints.orientation_constraints.clear();
        mp.request.motion_plan_req.goal_constraints.position_constraints.clear();
        poseToClopemaMotionPlan(mp, desired_poses[i]);
        pointToRobotState(mp.request.motion_plan_req.start_state, wholeTraj.points.back(), wholeTraj.joint_names);
        if (!plan(mp))
            return -1;
        wholeTraj.points.insert(wholeTraj.points.end(), mp.response.joint_trajectory.points.begin(), mp.response.joint_trajectory.points.end());

    }

    //ROS_INFO("Executing trajectory size: %d", wholeTraj.points.size());
    control_msgs::FollowJointTrajectoryGoal goal;
    goal.trajectory = wholeTraj;
    cmove.doGoal(goal);

    return 0;
}

//void grabFromXtion(cv::Mat rgb, cv::Mat depth, pcl::PointCloud<pcl::PointXYZ> pc){

//    camera_helpers::OpenNICaptureAll grabber("xtion3") ;
//    grabber.connect();
//    ros::Time ts(0);
//    image_geometry::PinholeCameraModel cm;
//    grabber.grab(rgb, depth, pc, ts, cm);

//    ros::Duration(1,0).sleep();
//}

float getArmsDistance(){

    tf::TransformListener listener;
    tf::StampedTransform transform;

    try {
        listener.waitForTransform("base_link", "r2_ee", ros::Time(0), ros::Duration(10.0) );
        listener.lookupTransform("base_link", "r2_ee", ros::Time(0), transform);
    } catch (tf::TransformException ex) {
        ROS_ERROR("%s",ex.what());
    }

    tf::Vector3 point1;
    point1.setX( transform.getOrigin().x() );
    point1.setY( transform.getOrigin().y() );
    point1.setZ( transform.getOrigin().z() );

    cout<<"point 1 " << point1.getX() << " " << point1.getY()<< " " << point1.getZ() << endl;

    try {
        listener.waitForTransform("base_link", "r1_ee", ros::Time(0), ros::Duration(10.0) );
        listener.lookupTransform("base_link", "r1_ee", ros::Time(0), transform);
    } catch (tf::TransformException ex) {
        ROS_ERROR("%s",ex.what());
    }

    tf::Vector3 point2;
    point2.setX( transform.getOrigin().x() );
    point2.setY( transform.getOrigin().y() );
    point2.setZ( transform.getOrigin().z() );

    cout<<"point 1 " << point2.getX() << " " << point2.getY()<< " " << point2.getZ() << endl;

    return tf::tfDistance(point1, point2);
}

geometry_msgs::Pose getArmPose( const string &armName, const string &frameName){

    geometry_msgs::Pose pose;
    tf::TransformListener listener;
    tf::StampedTransform transform;

    try {
        listener.waitForTransform(frameName, armName + "_ee", ros::Time(0), ros::Duration(10.0) );
        listener.lookupTransform(frameName, armName + "_ee", ros::Time(0), transform);
    } catch (tf::TransformException ex) {
        ROS_ERROR("%s",ex.what());
    }
    pose.position.x=transform.getOrigin().x();
    pose.position.y=transform.getOrigin().y();
    pose.position.z=transform.getOrigin().z();

    pose.orientation.x=transform.getRotation().x();
    pose.orientation.y=transform.getRotation().y();
    pose.orientation.z=transform.getRotation().z();
    pose.orientation.w=transform.getRotation().w();

    return pose;
}

Eigen::Matrix4d getTranformationMatrix(const string &frameName, const string &coordSys ){
    tf::TransformListener listener;
    ros::Time ts(0) ;
    tf::StampedTransform transform;
    Eigen::Affine3d pose;



    try {
        listener.waitForTransform( coordSys, frameName, ts, ros::Duration(1) );
        listener.lookupTransform( coordSys, frameName, ts, transform);
        tf::TransformTFToEigen(transform, pose);
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
    }

    return pose.matrix();
}

tf::StampedTransform  getTranformation( const string &frameName, const string &coordSys ){

    tf::TransformListener listener;
    ros::Time ts(0) ;
    tf::StampedTransform transform;


    try {
        listener.waitForTransform( coordSys, frameName, ts, ros::Duration(1) );
        listener.lookupTransform( coordSys, frameName, ts, transform);
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
    }

    return transform;
}

void publishLowestPointMarker(ros::Publisher &vis_pub, const Eigen::Vector3d &p, const Eigen::Vector3d &n)
{


    visualization_msgs::Marker marker;
        // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = "/xtion3_rgb_optical_frame";
    marker.header.stamp = ros::Time::now();

    marker.ns = "lowest point";
    marker.id = 0;

    // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
    marker.type = visualization_msgs::Marker::ARROW ;

    Eigen::Vector3d ep = p + 0.3 *n ;

    geometry_msgs::Point p1, p2 ;
    p1.x = p.x() ;
    p1.y = p.y() ;
    p1.z = p.z() ;
    p2.x = ep.x() ;
    p2.y = ep.y() ;
    p2.z = ep.z() ;

    marker.points.push_back(p1) ;
    marker.points.push_back(p2) ;

    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;

    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;

    marker.lifetime = ros::Duration();

    vis_pub.publish(marker);

}


}
