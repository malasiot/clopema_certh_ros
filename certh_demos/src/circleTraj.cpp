#include "clopema_motoros/ClopemaMove.h"
#include <clopema_motoros/WriteIO.h>

using namespace std;

/*
 * Convert joint trajectory point to robot state
 */
void pointToRobotState(arm_navigation_msgs::RobotState &state, trajectory_msgs::JointTrajectoryPoint &point, std::vector<std::string> & joint_names) {
    for (unsigned int i = 0; i < joint_names.size(); i++) {
        for (unsigned int j = 0; j < state.joint_state.name.size(); j++) {
            if (state.joint_state.name.at(j) == joint_names.at(i)) {
                state.joint_state.position.at(j) = point.positions.at(i);
                break;
            }
        }
    }
}

std::vector<geometry_msgs::Pose> calculate_circle_points(float x, float y, float z , float r) {
   
    std::vector<geometry_msgs::Pose> points;
    float tmp_x,tmp_z;
    geometry_msgs::Pose point;
    point.position.x=x;
    point.position.y=y;
    point.position.z=z;
    point.orientation = tf::createQuaternionMsgFromRollPitchYaw(M_PI, 0, 0);


    float step = 3.14+3.14/2;
    for (unsigned int i = 0; i <5; i ++){
        
        tmp_x=r*cos(step);
        tmp_z=r*sin(step);
        point.position.x=x+tmp_x;
        point.position.y=y;
        point.position.z=z+tmp_z;
        cout<< "x= "<<point.position.x<<" "<< "y= "<<point.position.y<<" "<< "z= "<<point.position.z<<"\n";
        point.orientation = tf::createQuaternionMsgFromRollPitchYaw(M_PI, 0, 0);
        points.push_back(point);
        step-=3.14 / 8 ;
    }
	
    return points;
}




void setPathConstraints(clopema_arm_navigation::ClopemaMotionPlan & mp) {


	mp.request.motion_plan_req.path_constraints.position_constraints.resize(1);
	mp.request.motion_plan_req.path_constraints.position_constraints[0].header.frame_id = "r2_ee";
	mp.request.motion_plan_req.path_constraints.position_constraints[0].header.stamp = ros::Time::now();
	mp.request.motion_plan_req.path_constraints.position_constraints[0].link_name = "r1_ee";
	mp.request.motion_plan_req.path_constraints.position_constraints[0].position.x = 0.0;
	mp.request.motion_plan_req.path_constraints.position_constraints[0].position.y = 0.0;
	mp.request.motion_plan_req.path_constraints.position_constraints[0].position.z = 0.0;
	mp.request.motion_plan_req.path_constraints.position_constraints[0].constraint_region_shape.type = arm_navigation_msgs::Shape::SPHERE;
	mp.request.motion_plan_req.path_constraints.position_constraints[0].constraint_region_shape.dimensions.push_back(0.65); //radius
	mp.request.motion_plan_req.path_constraints.position_constraints[0].constraint_region_shape.dimensions.push_back(0.65);
	mp.request.motion_plan_req.path_constraints.position_constraints[0].constraint_region_shape.dimensions.push_back(0.65);
	mp.request.motion_plan_req.path_constraints.position_constraints[0].constraint_region_orientation.x = 0.0;
	mp.request.motion_plan_req.path_constraints.position_constraints[0].constraint_region_orientation.y = 0.0;
	mp.request.motion_plan_req.path_constraints.position_constraints[0].constraint_region_orientation.z = 0.0;
	mp.request.motion_plan_req.path_constraints.position_constraints[0].constraint_region_orientation.w = 1.0;
	mp.request.motion_plan_req.path_constraints.position_constraints[0].weight = 1.0;

}

void setGrippersOpen(){
	
	ros::service::waitForService("/write_io");
	clopema_motoros::WriteIO sopen;
	sopen.request.address=10022;
	sopen.request.value=false;
	ros::service::call("/write_io", sopen);
	sopen.request.value=true;
	ros::service::call("/write_io", sopen);
	return;

}
void setGrippersClose(){
	
	ros::service::waitForService("/write_io");
	clopema_motoros::WriteIO sopen;
	sopen.request.address=10023;
	sopen.request.value=false;
	ros::service::call("/write_io", sopen);	
	sopen.request.value=true;
	ros::service::call("/write_io", sopen);	
	
}

/**
 * The following code will plan trajectory through few points
 * and then whole trajectory will be executed at once.
 */

int main(int argc, char **argv) {
    ros::init(argc, argv, "more_points");
    ros::NodeHandle nh;
    std::vector<geometry_msgs::Pose> traj;

    ClopemaMove cmove;
    trajectory_msgs::JointTrajectory wholeTraj;
   
    //Create plan
    clopema_arm_navigation::ClopemaMotionPlan mp;
    mp.request.motion_plan_req.group_name = "r1_arm";
    mp.request.motion_plan_req.allowed_planning_time = ros::Duration(5.0);
    if (!cmove.getRobotState(mp.request.motion_plan_req.start_state))
        return -1;

    arm_navigation_msgs::SimplePoseConstraint desired_pose;
    desired_pose.header.frame_id = "base_link";
    desired_pose.header.stamp = ros::Time::now();
    desired_pose.link_name = "r1_ee";
    desired_pose.pose.position.x = 0.0;
    desired_pose.pose.position.y = -1;
    desired_pose.pose.position.z = 0.9;
    desired_pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(3.14/2, 3.14/2, 3.14/2);
    desired_pose.absolute_position_tolerance.x = 0.02;
    desired_pose.absolute_position_tolerance.y = 0.02;
    desired_pose.absolute_position_tolerance.z = 0.02;
    desired_pose.absolute_roll_tolerance = 0.04;
    desired_pose.absolute_pitch_tolerance = 0.04;
    desired_pose.absolute_yaw_tolerance = 0.04;
	setPathConstraints(mp);
	
    cmove.poseToClopemaMotionPlan(mp, desired_pose);
    if (!cmove.plan(mp))
        return -1;
    wholeTraj = mp.response.joint_trajectory;
    traj=calculate_circle_points(0,-1,1.5,0.6);
    setGrippersOpen();
    
    //remove goal constraints and set new one
    for(unsigned int i=0;i<traj.size();i++){
		if(i==traj.size()-1){
			mp.request.motion_plan_req.goal_constraints.orientation_constraints.clear();
			mp.request.motion_plan_req.goal_constraints.position_constraints.clear();
			desired_pose.pose.position.x = traj[i].position.x;
			desired_pose.pose.position.z = traj[i].position.z;
			desired_pose.pose.orientation.x = 1;
			desired_pose.pose.orientation.y = 0;
			desired_pose.pose.orientation.z = 0;
			desired_pose.pose.orientation.w = 0;
			
			cmove.poseToClopemaMotionPlan(mp, desired_pose);
			pointToRobotState(mp.request.motion_plan_req.start_state, wholeTraj.points.back(), wholeTraj.joint_names);
			if (!cmove.plan(mp))
				return -1;
			wholeTraj.points.insert(wholeTraj.points.end(), mp.response.joint_trajectory.points.begin(), mp.response.joint_trajectory.points.end());
			}
		else{
			mp.request.motion_plan_req.goal_constraints.orientation_constraints.clear();
			mp.request.motion_plan_req.goal_constraints.position_constraints.clear();
			desired_pose.pose.position.x = traj[i].position.x;
			desired_pose.pose.position.z = traj[i].position.z;
			cmove.poseToClopemaMotionPlan(mp, desired_pose);
			pointToRobotState(mp.request.motion_plan_req.start_state, wholeTraj.points.back(), wholeTraj.joint_names);
			if (!cmove.plan(mp))
				return -1;
			wholeTraj.points.insert(wholeTraj.points.end(), mp.response.joint_trajectory.points.begin(), mp.response.joint_trajectory.points.end());
		}
		if(i==1)
			setGrippersClose();
    }

    //~ //to the home position
    //~ mp.request.motion_plan_req.goal_constraints.orientation_constraints.clear();
    //~ mp.request.motion_plan_req.goal_constraints.position_constraints.clear();
    //~ vector<string> joint_names = cmove.getJointsInGroup(mp.request.motion_plan_req.group_name);
    //~ mp.request.motion_plan_req.goal_constraints.joint_constraints.resize(joint_names.size());
    //~ for (unsigned int i = 0; i < mp.request.motion_plan_req.goal_constraints.joint_constraints.size(); ++i) {
        //~ mp.request.motion_plan_req.goal_constraints.joint_constraints[i].joint_name = joint_names[i];
        //~ mp.request.motion_plan_req.goal_constraints.joint_constraints[i].position = 0.0;
        //~ mp.request.motion_plan_req.goal_constraints.joint_constraints[i].tolerance_below = 0.1;
        //~ mp.request.motion_plan_req.goal_constraints.joint_constraints[i].tolerance_above = 0.1;
    //~ }
    //~ pointToRobotState(mp.request.motion_plan_req.start_state, wholeTraj.points.back(), wholeTraj.joint_names);
    //~ if (!cmove.plan(mp))
        //~ return -1;
    //~ wholeTraj.points.insert(wholeTraj.points.end(), mp.response.joint_trajectory.points.begin(), mp.response.joint_trajectory.points.end());

    ROS_INFO("Executing trajectory size: %d", wholeTraj.points.size());
    control_msgs::FollowJointTrajectoryGoal goal;
    goal.trajectory = wholeTraj;
    cmove.doGoal(goal);

    ros::shutdown();
    return 1;
}
