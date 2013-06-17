#include "robot_helpers/Utils.h"

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

bool moveGripperPointingDown(MoveRobot &cmove, const std::string &armName, double X, double Y, double Z)
{

    clopema_arm_navigation::ClopemaMotionPlan mp;
    mp.request.motion_plan_req.group_name = armName + "_arm" ;
    mp.request.motion_plan_req.allowed_planning_time = ros::Duration(5.0);


    if ( !getRobotState(mp.request.motion_plan_req.start_state) ) return false ;

    arm_navigation_msgs::SimplePoseConstraint desired_pose;

    desired_pose.header.frame_id = "base_link";
    desired_pose.header.stamp = ros::Time::now();
    desired_pose.link_name = armName + "_ee";
    desired_pose.pose.position.x = X ;
    desired_pose.pose.position.y = Y ;
    desired_pose.pose.position.z = Z ;

    Eigen::Quaterniond q ;
    q = Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX());

    desired_pose.pose.orientation.x = q.x() ;
    desired_pose.pose.orientation.y = q.y() ;
    desired_pose.pose.orientation.z = q.z() ;
    desired_pose.pose.orientation.w = q.w() ;

    desired_pose.absolute_position_tolerance.x = desired_pose.absolute_position_tolerance.y = desired_pose.absolute_position_tolerance.z = 0.02;
    desired_pose.absolute_roll_tolerance = desired_pose.absolute_pitch_tolerance = desired_pose.absolute_yaw_tolerance = 0.04;

    poseToClopemaMotionPlan(mp, desired_pose);

    ROS_INFO("Planning to the start position");

    if (!plan(mp)) return false;

    trajectory_msgs::JointTrajectory wholeTraj = mp.response.joint_trajectory;
    mp.response.joint_trajectory = wholeTraj;

    control_msgs::FollowJointTrajectoryGoal goal;

    if ( mp.response.joint_trajectory.points.size() > 2 )
    {
        goal.trajectory = mp.response.joint_trajectory;
        cmove.doGoal(goal) ;

    }

    return true ;
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























}
