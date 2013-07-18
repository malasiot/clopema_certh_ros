#include "robot_helpers/Unfold.h"
#include <opencv2/highgui/highgui.hpp>

using namespace std;
namespace robot_helpers{

Unfold::Unfold(const string &armName){
    setHoldingArm(armName);
}

Unfold::~Unfold() {
}

bool Unfold::moveHomeArm(const string &armName){

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

int Unfold::moveArm(geometry_msgs::Pose pose, const string &armName){

        //Create plan
        clopema_arm_navigation::ClopemaMotionPlan mp;
        MoveRobot cmove;


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


int Unfold::moveArms( geometry_msgs::Pose pose1, geometry_msgs::Pose pose2, const string &arm1Name, const string &arm2Name){

    MoveRobot cmove;

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

}


int Unfold::moveArmsNoTearing( geometry_msgs::Pose pose1, geometry_msgs::Pose pose2, const string &arm1Name, const string &arm2Name){

    float radious = getArmsDistance()+0.1;

    MoveRobot cmove;
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


}

int Unfold::moveArmThrough(vector <geometry_msgs::Pose> poses , const string &armName){


    MoveRobot cmove;
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

    ROS_INFO("Executing trajectory size: %d", wholeTraj.points.size());
    control_msgs::FollowJointTrajectoryGoal goal;
    goal.trajectory = wholeTraj;
    cmove.doGoal(goal);

}


int Unfold::setGripperStates(const string &armName  , bool open){
    ros::service::waitForService("/" + armName + "_gripper/set_open");
    clopema_motoros::SetGripperState sopen;
    sopen.request.open=open;
    ros::service::call("/" + armName + "_gripper/set_open", sopen);
}

int Unfold::setGrippersStates( bool open){
    ros::service::waitForService("/r1_gripper/set_open");
    clopema_motoros::SetGripperState sopen;
    sopen.request.open=open;
    ros::service::call("/r1_gripper/set_open", sopen);

    ros::service::waitForService("/r2_gripper/set_open");
    sopen.request.open=open;
    ros::service::call("/r2_gripper/set_open", sopen);
}

float Unfold::getArmsDistance(){

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

geometry_msgs::Pose Unfold::getArmPose( const string &armName, const string &frameName){

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

void Unfold::setPathConstraints(clopema_arm_navigation::ClopemaMotionPlan & mp, float radious , const string &armName, geometry_msgs::Quaternion q ) {

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


int Unfold::moveArmConstrains(geometry_msgs::Pose pose, const string &armName, float radious){
        //Create plan
        clopema_arm_navigation::ClopemaMotionPlan mp;
        MoveRobot cmove;

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

        setPathConstraints(mp, radious, armName, rotationMatrix3ToQuaternion(vertical()));

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



void Unfold::rotateGripper(float angle ,const string &armName){

    //Create plan
    clopema_arm_navigation::ClopemaMotionPlan mp;
    MoveRobot cmove;

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


void Unfold::rotateHoldingGripper(float angle){

    //Create plan
    clopema_arm_navigation::ClopemaMotionPlan mp;
    MoveRobot cmove;

    mp.request.motion_plan_req.group_name = holdingArm + "_arm";
    mp.request.motion_plan_req.allowed_planning_time = ros::Duration(5.0);

    //Set start state
    getRobotState(mp.request.motion_plan_req.start_state);

    arm_navigation_msgs::SimplePoseConstraint desired_pose;

    desired_pose.header.frame_id = holdingArm + "_ee";
    desired_pose.header.stamp = ros::Time::now();
    desired_pose.link_name = holdingArm + "_ee";

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

void Unfold::switchArms(){

    string tmp=holdingArm;

    holdingArm=movingArm;
    movingArm=tmp;

}

void Unfold::setHoldingArm(const string &armName){

    holdingArm = armName;
    if(armName == "r1")
        movingArm = "r2";    
    else
        movingArm = "r1";

}


Eigen::Matrix4d Unfold::findGraspingOrientation(Eigen::Vector4d vector ){

    Eigen::Matrix4d rotMat;
    Eigen::Vector3d N;

    rotMat(0, 0) = vector.x();
    rotMat(0, 1) = vector.y();
    rotMat(0, 2) = vector.z();

    N << vector.x(), vector.y(), vector.z();
    if(holdingArm == "r2"){
        rotMat(0, 0) = -vector.x();
        rotMat(0, 1) = -vector.y();
        rotMat(0, 2) = -vector.z();

        N << -vector.x(), -vector.y(), -vector.z();
    }
    N.normalize();

    Eigen::Vector3d A(1,0,-1);
    if(holdingArm == "r2")
         A << -1,0,-1;

    Eigen::Vector3d B=A-(A.dot(N))*N;
    Eigen::Vector3d C=N.cross(B);

    C.normalize();
    B.normalize();

    rotMat(0, 0)=N.x();
    rotMat(1, 0)=N.y();
    rotMat(2, 0)=N.z();

    rotMat(0, 1)=C.x();
    rotMat(1, 1)=C.y();
    rotMat(2, 1)=C.z();

    rotMat(0, 2)=-B.x();
    rotMat(1, 2)=-B.y();
    rotMat(2, 2)=-B.z();

return rotMat;
}

float Unfold::findBias(Eigen::Vector4d vector){

    float theta=atan2f(vector.x(),abs(vector.y()));

    return theta;
}

geometry_msgs::Quaternion Unfold::rotationMatrix4ToQuaternion(Eigen::Matrix4d matrix){

    float roll , pitch, yaw;

    roll= atan2f(matrix(2, 1),matrix(2, 2) );
    pitch= atan2f(-matrix(2,0),sqrt(pow(matrix(2, 2),2)+pow(matrix(2, 1),2)));
    yaw= atan2f(matrix(1, 0),matrix(0, 0));
    return tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, yaw );
}

geometry_msgs::Quaternion Unfold::rotationMatrix3ToQuaternion(Eigen::Matrix3d matrix){

    float roll , pitch, yaw;

    roll= atan2f(matrix(2, 1),matrix(2, 2) );
    pitch= atan2f(-matrix(2,0),sqrt(pow(matrix(2, 2),2)+pow(matrix(2, 1),2)));
    yaw= atan2f(matrix(1, 0),matrix(0, 0));
    return tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, yaw );
}

Eigen::Matrix4d Unfold::getTranformationMatrix(const string &frameName, const string &coordSys ){
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

tf::StampedTransform  Unfold::getTranformation( const string &frameName, const string &coordSys ){

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

void mouse_callback( int event, int x, int y, int flags, void* param){
    if(event == CV_EVENT_LBUTTONDOWN){
        cout << x << " " << y << endl;
    }
}


void Unfold::robustPlane3DFit(vector<Eigen::Vector3f> &x, Eigen::Vector3f  &c, Eigen::Vector3f &u)
{
    int i, k, N = x.size() ;
    Eigen::Vector3f u1, u2, u3 ;
    const int NITER = 5 ;

    double *weight = new double [N] ;
    double *res = new double [N] ;

    for( i=0 ; i<N ; i++ ) weight[i] = 1.0 ;

    double wsum = N ;

    for( k=0 ; k<NITER ; k++ )
    {

        c = Eigen::Vector3f::Zero() ;
        Eigen::Matrix3f cov = Eigen::Matrix3f::Zero();

        for( i=0 ; i<N ; i++ )
        {
            const Eigen::Vector3f &P = x[i] ;
            double w = weight[i] ;

            c += w * P ;
        }

        c /= wsum ;

        for( i=0 ; i<N ; i++ )
        {
            const Eigen::Vector3f &P = x[i] ;
            double w = weight[i] ;

            cov += w *  (P - c) * (P - c).adjoint();

        }

        cov *= 1.0/wsum ;

        Eigen::Matrix3f U ;
        Eigen::Vector3f L ;

        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigensolver(cov);
        L = eigensolver.eigenvalues() ;
        U = eigensolver.eigenvectors() ;

        // Recompute weights

        u1 = U.col(0) ;
        u2 = U.col(1) ;
        u3 = U.col(2) ;

        for( i=0 ; i<N ; i++ )
        {
            const Eigen::Vector3f &P = x[i] ;

        //    double r = (P -c).dot(P-c) ;
        //    double ss = (P-c).dot(u1);

         //   r -= ss * ss ;

          //  r = sqrt(r) ;

            double r = fabs((P-c).dot(u1));

            weight[i] = r ;
            res[i] = r ;
        }

        // Estimate residual variance
        double sigma ;

        sort(weight, weight + N) ;

        sigma = weight[N/2]/0.6745 ;

        // Update weights using Hubers scheme

        wsum = 0.0 ;

        for( i=0 ; i<N ; i++ )
        {

            double r = fabs(res[i]) ;

            if ( r <= sigma )  weight[i] = 1.0 ;
            else if ( r > sigma && r <= 3.0*sigma ) weight[i] = sigma/r ;
            else weight[i] = 0.0 ;

            wsum += weight[i] ;
        }
    }

    u = u1 ;

    delete weight ;
    delete res ;
}

Eigen::Vector3f Unfold::computeNormal(const pcl::PointCloud<pcl::PointXYZ> &pc, int x, int y){
    const int nrmMaskSize = 7 ;
    int w = pc.width, h = pc.height ;

    vector<Eigen::Vector3f> pts ;

    for(int i = y - nrmMaskSize ; i<= y + nrmMaskSize ; i++  )
        for(int j = x - nrmMaskSize ; j<= x + nrmMaskSize ; j++  )
        {
            if ( i < 0 || j < 0 || i > h-1 || j > w-1 ) continue ;

            pcl::PointXYZ val = pc.at(j, i) ;

            if ( !pcl_isfinite(val.z) ) continue ;

            pts.push_back(Eigen::Vector3f(val.x, val.y, val.z)) ;
        }

    if ( pts.size() < 3 ) return Eigen::Vector3f() ;

    Eigen::Vector3f u, c ;
    robustPlane3DFit(pts, c, u);

    if ( u(2) < 0 ) u = -u ;

    return u ;

}


bool Unfold::findLowestPoint(const pcl::PointCloud<pcl::PointXYZ> &depth, const Eigen::Vector3f &orig, const Eigen::Vector3f &base, float apperture, Eigen::Vector3f &p, Eigen::Vector3f &n, cv::Mat depthMap){

    int w = depth.width, h = depth.height ;

    int best_i=-1, best_j=-1;
    float best_x=-1, best_y=-1, best_z=-1;


    bool found = false ;
    float minx = 10 ;
    for(int j=66 ; j<626 ; j++ )
    {


        for(int i=100 ; i<350 ; i++)
        {
            pcl::PointXYZ val = depth.at(j, i) ;

            if ( val.z<1 || val.z>1.5 ) continue ;

            if ( minx>val.x )
            {
                minx = val.x ;
                best_j = j ;
                best_i = i ;

                found = true ;
            }

        }
    }



    for(int i=0; i<depthMap.rows; ++i)
        for(int j=0; j<depthMap.cols; ++j)
            if( (depthMap.at<unsigned short>(i, j) < 900) || (depthMap.at<unsigned short>(i, j) > 1400) )
                depthMap.at<unsigned short>(i, j) = 0;

    cv::TermCriteria criteria(1, 10, 0.1);
    cv::Rect rect1(best_j-10, best_i-10, 20, 20);
    cv::meanShift(depthMap, rect1, criteria);
    best_j = rect1.x + rect1.width/2;
    best_i = rect1.y + rect1.height/2;
//    cout<< "best2= (" << best_j <<" , "<< best_i <<")"<<endl;

    pcl::PointXYZ p_ = depth.at(best_j, best_i ) ;
    n = computeNormal(depth, best_j, best_i ) ;
    p = Eigen::Vector3f(p_.x, p_.y, p_.z) ;

    if ( !found ) return false ;

}


int Unfold::moveArmBetweenSpheres( string armName, bool up, geometry_msgs::Pose goalPose){

    float radious = getArmsDistance();

    tf::StampedTransform st = getTranformation(holdingArm + "_ee");
    geometry_msgs::Point goalPoint;
    goalPoint.x = st.getOrigin().x();
    goalPoint.y = st.getOrigin().y();
    goalPoint.z = st.getOrigin().z();

    if(movingArm == "r2"){
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
    cout<< "CIRCLE" << endl;
    printPose( goalPose);
    addSphereToCollisionModel(holdingArm, radious/2.0);

    if ( moveArmConstrains( goalPose, armName, radious + 0.1) == -1){
        cout<<"ABORDING..." <<endl;
        return -1;
    }

    resetCollisionModel();

}

void printPose(geometry_msgs::Pose p){

    cout << "pose = (" << p.position.x << ", "<< p.position.y << ", "<< p.position.z << " )" <<endl;

}




void Unfold::grabFromXtion(cv::Mat rgb, cv::Mat depth, pcl::PointCloud<pcl::PointXYZ> pc){

    camera_helpers::OpenNICaptureAll grabber("xtion3") ;
    grabber.connect();
    ros::Time ts(0);
    image_geometry::PinholeCameraModel cm;
    grabber.grab(rgb, depth, pc, ts, cm);

    ros::Duration(1,0).sleep();

    cv::namedWindow("rgb");
    cv::setMouseCallback("rgb", mouse_callback, NULL);

    int k=0;
    while(k!=1048603){
        grabber.grab(rgb, depth, pc, ts, cm);
        cv::imshow("rgb", rgb);
        k = cv::waitKey(1);
    }

}



int Unfold::GraspLowestPoint(bool lastMove){


    moveArms(movingArmPose(), holdingArmPose(), movingArm, holdingArm );

    setGripperStates(movingArm , true);
    ros::Duration(1.0).sleep();

    //starting position
    bool grasp = false;
    int tries = 0;
    camera_helpers::OpenNICaptureAll grabber("xtion3") ;
    geometry_msgs::Pose desPose;
    vector <geometry_msgs::Pose> poses;
    Eigen::Vector4d targetP;
    Eigen::Matrix4d rotMat;
    grabber.connect() ;


    while(!grasp || tries<5){

        cv::Mat rgb, depth;
        pcl::PointCloud<pcl::PointXYZ> pc;
        cv::Mat R = cv::Mat(4, 4, CV_32FC1, cv::Scalar::all(0));
        ros::Time ts(0);
        image_geometry::PinholeCameraModel cm;

        //grabbing image and making the fit of lowest point
        Eigen::Matrix4d calib = getTranformationMatrix("xtion3_rgb_optical_frame");

        tf::StampedTransform st;
        Eigen::Vector3f top , bottom, p, n;
        float angle=0.2;

        st= getTranformation("xtion3_rgb_optical_frame", holdingArm + "_ee");

        top.x()=st.getOrigin().x();
        top.y()=st.getOrigin().y();
        top.z()=st.getOrigin().z();
        bottom = top;
        bottom.x()-=1.2;

        grabber.grab(rgb, depth, pc, ts, cm);

        findLowestPoint(pc, top, bottom, angle, p, n, depth);

        Eigen::Vector4d tar(p.x(), p.y(), p.z(), 1);
        targetP = calib * tar;

        Eigen::Vector4d norm (n.x(), n.y(), n.z(), 0);
        Eigen::Vector4d targetN;
        targetN = calib * norm.normalized();

        rotMat = findGraspingOrientation(targetN);

        desPose.orientation = rotationMatrix4ToQuaternion(rotMat);
        desPose.position.x = targetP.x() + rotMat(0, 0) * 0.03 - rotMat(0, 2) * 0.07;
        desPose.position.y = targetP.y() + rotMat(1, 0) * 0.03 - rotMat(1, 2) * 0.07;
        desPose.position.z = targetP.z() + rotMat(2, 0) * 0.03 - rotMat(2, 2) * 0.07;
        poses.push_back(desPose);
        desPose.position.x = targetP.x() + rotMat(0, 2) * 0.045 + rotMat(0, 0) * 0.03;
        desPose.position.y = targetP.y() + rotMat(1, 2) * 0.045 + rotMat(1, 0) * 0.03;
        desPose.position.z = targetP.z() + rotMat(2, 2) * 0.045 + rotMat(2, 0) * 0.03;
        poses.push_back(desPose);
        st= getTranformation(holdingArm + "_ee");

        //grasp lowest point
        addConeToCollisionModel(holdingArm, st.getOrigin().z() - desPose.position.z - 0.2, 0.1 );

        if(moveArmThrough(poses, movingArm) == -1){
            resetCollisionModel();
            cout<< "BIAS : " <<findBias(targetN)<<endl;
            rotateGripper(findBias(targetN), holdingArm);
            tries++;
            continue;
        }
        resetCollisionModel();
        break;

    }
    setGripperStates( movingArm, false);

    //Make a circle trajectory
    desPose.orientation = rotationMatrix3ToQuaternion(vertical());

    geometry_msgs::Pose desPos1, desPos2;
    float radious=getArmsDistance();

    desPos1 = getArmPose("r1");
    desPos2 = getArmPose("r2");

    if(holdingArm == "r1"){
        desPos2.position.x-=radious/2.0;
        desPos1.position.x-=radious/2.0;


    }
    else{
        desPos1.position.x+=radious/2.0;
        desPos2.position.x+=radious/2.0;

    }

    ros::Duration(1.0).sleep();
    moveArmsNoTearing(desPos1, desPos2);

    ros::Duration(1.0).sleep();
    moveArmBetweenSpheres(movingArm, true,  desPose);
    if ( lastMove == true )
        return -1;
    desPos1 = getArmPose("r1");
    desPos2 = getArmPose("r2");


    cout<< "holding arm is = " << holdingArm << endl;
    printPose( desPos1);
    printPose( desPos2);
    if(holdingArm == "r1"){
        desPos2.position.x-=radious/2.0f;
        desPos1.position.z-=0.866025404*radious;
    }
    else{
        desPos1.position.x+=radious/2.0f;
        desPos2.position.z-=0.866025404*radious;
    }

    ros::Duration(1.5).sleep();
    if ( moveArms(desPos1, desPos2) == -1){
        cout<<"ABORDING..."<<endl;
        return -1;
    }

    setGripperStates(holdingArm, true);

    switchArms();


return 0;

}




}

