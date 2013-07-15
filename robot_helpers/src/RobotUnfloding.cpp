#include "robot_helpers/Unfold.h"


#include <opencv2/highgui/highgui.hpp>
namespace robot_helpers{

Unfold::Unfold(){

}

Unfold::~Unfold() {
}



int Unfold::moveArm(geometry_msgs::Pose pose, const std::string &armName){

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

int Unfold::moveArms( geometry_msgs::Pose pose1, geometry_msgs::Pose pose2){

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
    desired_pose.link_name = "r2_ee";
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
    desired_pose.link_name = "r1_ee";
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

int Unfold::setGripperStates(const std::string &armName  , bool open){
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

    std::cout<<"point 1 " << point1.getX() << " " << point1.getY()<< " " << point1.getZ() << std::endl;

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

    std::cout<<"point 1 " << point2.getX() << " " << point2.getY()<< " " << point2.getZ() << std::endl;

    return tf::tfDistance(point1, point2);
}

geometry_msgs::Pose Unfold::getArmPose( const std::string &armName, const std::string &frameName){

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

void Unfold::setPathConstraints(clopema_arm_navigation::ClopemaMotionPlan & mp, float radious , const std::string &armName, geometry_msgs::Quaternion q ) {

    std::string arm2Name="r1";
    if (armName == "r1")
        arm2Name="r2";

    mp.request.motion_plan_req.path_constraints.position_constraints.resize(1);
    mp.request.motion_plan_req.path_constraints.position_constraints[0].header.frame_id = armName + "_ee";
    mp.request.motion_plan_req.path_constraints.position_constraints[0].header.stamp = ros::Time::now();
    mp.request.motion_plan_req.path_constraints.position_constraints[0].link_name = arm2Name + "_ee";
    mp.request.motion_plan_req.path_constraints.position_constraints[0].position.x = 0.0;
    mp.request.motion_plan_req.path_constraints.position_constraints[0].position.y = 0.0;
    mp.request.motion_plan_req.path_constraints.position_constraints[0].position.z = 0.0;
    mp.request.motion_plan_req.path_constraints.position_constraints[0].constraint_region_orientation= q;
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


int Unfold::moveArmConstrains(geometry_msgs::Pose pose, const std::string &armName, float radious, geometry_msgs::Quaternion q){
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



void Unfold::rotateGripper(float angle ,const std::string &armName){

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

void Unfold::switchArms(){

    std::string tmp=holdingArm;

    holdingArm=movingArm;
    movingArm=tmp;

}

void Unfold::setHoldingArm(const std::string &armName){

    if(armName == "r1"){
        holdingArm = armName;
        movingArm = "r2";
    }
    else{
        holdingArm = armName;
        movingArm = "r1";
    }

}


void Unfold::findGraspingOrientation(Eigen::Vector4d vector, Eigen::Matrix4d rotMat){

    Eigen::Vector3d N(vector.x(), vector.y(), vector.z());
    N.normalize();
    Eigen::Vector3d A(1,0,-1);
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

//    cout<<N<<"\n"<<C<<"\n "<<B<<endl;
//    cout<<C.dot(N)<< " "<<B.dot(N)<< endl;

}

float Unfold::findBias(Eigen::Vector4d vector){

    float theta=atan2f(vector.x(),abs(vector.y()));

    return theta;
}

geometry_msgs::Quaternion Unfold::rotationMatrixToQuaternion(btMatrix3x3 matrix){

    float roll , pitch, yaw;

    roll= atan2f(matrix[2][1],matrix[2][2] );
    pitch= atan2f(-matrix[2][0],sqrt(pow(matrix[2][2],2)+pow(matrix[2][1],2)));
    yaw= atan2f(matrix[1][0],matrix[0][0]);
    return tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, yaw );
}



Eigen::Matrix4d Unfold::getTranformationMatrix(const std::string &frameName, const std::string &coordSys ){
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

tf::StampedTransform  Unfold::getTranformation( const std::string &frameName, const std::string &coordSys ){

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


int Unfold::GraspLowestPoint(const std::string &armName){


    setHoldingArm(armName);
    setGripperStates(movingArm , true);
    ros::Duration(1.0).sleep();

    camera_helpers::OpenNICaptureAll grabber("xtion3") ;
    grabber.connect() ;

    cv::Mat rgb, depth;
    pcl::PointCloud<pcl::PointXYZ> pc;
    cv::Mat R = cv::Mat(4, 4, CV_32FC1, cv::Scalar::all(0));
    ros::Time ts(0);
    image_geometry::PinholeCameraModel cm;

    Eigen::Matrix4d calib = Unfold::getTranformationMatrix("xtion3_rgb_optical_frame");

    grabber.grab(rgb, depth, pc, ts, cm);





return 0;

}




}
