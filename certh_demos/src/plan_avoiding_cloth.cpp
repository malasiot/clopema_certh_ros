#include "ros/ros.h"


#include <robot_helpers/Robot.h>
#include <robot_helpers/Utils.h>

#include <Eigen/Geometry>

#include <tf_conversions/tf_eigen.h>

using namespace robot_helpers ;
using namespace std;
using namespace Eigen;

bool moveArmHoriz(MoveRobot &cmove, const std::string &armName, double X, double Y, double Z, double theta = 0.0)
{


    Eigen::Quaterniond q ;

    if ( armName == "r2" )
        q = Eigen::AngleAxisd(theta, Eigen::Vector3d::UnitX()) * Eigen::AngleAxisd(-M_PI/2, Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(-M_PI/2, Eigen::Vector3d::UnitZ());
    else
        q = Eigen::AngleAxisd(theta, Eigen::Vector3d::UnitX()) * Eigen::AngleAxisd(M_PI/2, Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(M_PI/2, Eigen::Vector3d::UnitZ());

    return moveGripper(cmove, armName, Eigen::Vector3d(X, Y, Z), q) ;
}

bool moveArmVert(MoveRobot &cmove, const std::string &armName, double X, double Y, double Z, double theta = 0.0)
{


    Eigen::Quaterniond q ;

       q =  Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()) * Eigen::AngleAxisd(theta, Eigen::Vector3d::UnitZ());

    return moveGripper(cmove, armName, Eigen::Vector3d(X, Y, Z), q) ;
}

bool planArmToPoseWithRollConstraint(const string &armName, const Eigen::Vector3d &pos, const Eigen::Quaterniond &q, trajectory_msgs::JointTrajectory &traj)
{
 /*   Affine3d pose = robot_helpers::getPose(armName) ;

    Quaterniond q0 ;
    q0 = pose.rotation() ;
    tf::Quaternion q0_, q1_ ;

    tf::RotationEigenToTF(q0, q0_) ;
    double yaw0, pitch0, roll0 ;
    btMatrix3x3(q0_.asBt()).getEulerZYX(yaw0, pitch0, roll0);

    tf::RotationEigenToTF(q, q1_) ;
    double yaw1, pitch1, roll1 ;
    btMatrix3x3(q1_.asBt()).getEulerZYX(yaw1, pitch1, roll1);

    tf::Quaternion goal = tf::createQuaternionFromRPY(roll0, pitch1, yaw1) ;
*/
    clopema_arm_navigation::ClopemaMotionPlan mp;
    mp.request.motion_plan_req.group_name = armName + "_arm" ;
    mp.request.motion_plan_req.allowed_planning_time = ros::Duration(5.0);

    if ( !getRobotState(mp.request.motion_plan_req.start_state) ) return false ;

    arm_navigation_msgs::SimplePoseConstraint desired_pose;

    mp.request.motion_plan_req.path_constraints.orientation_constraints.resize(1) ;

    mp.request.motion_plan_req.path_constraints.orientation_constraints[0].header.frame_id = armName + "_link_5";
    mp.request.motion_plan_req.path_constraints.orientation_constraints[0].header.stamp = ros::Time::now();
    mp.request.motion_plan_req.path_constraints.orientation_constraints[0].link_name = armName + "_ee";

    mp.request.motion_plan_req.path_constraints.orientation_constraints[0].orientation.x = 0;
    mp.request.motion_plan_req.path_constraints.orientation_constraints[0].orientation.y = 0;
    mp.request.motion_plan_req.path_constraints.orientation_constraints[0].orientation.z = 0;
    mp.request.motion_plan_req.path_constraints.orientation_constraints[0].orientation.w = 1;
    mp.request.motion_plan_req.path_constraints.orientation_constraints[0].type = arm_navigation_msgs::OrientationConstraint::HEADER_FRAME;
    mp.request.motion_plan_req.path_constraints.orientation_constraints[0].absolute_roll_tolerance = 0.2;
    mp.request.motion_plan_req.path_constraints.orientation_constraints[0].absolute_pitch_tolerance = M_PI;
    mp.request.motion_plan_req.path_constraints.orientation_constraints[0].absolute_yaw_tolerance = M_PI;


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

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "plan_avoiding_cloth") ;
    ros::NodeHandle nh ;

    MoveRobot mv ;

     double theta = -M_PI/4 ;

     Quaterniond q0, q1 ;
     q0 =  Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()) * Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX()) * Eigen::AngleAxisd(theta, Eigen::Vector3d::UnitZ());
     q1 =  Eigen::AngleAxisd(M_PI/2, Eigen::Vector3d::UnitX()) * Eigen::AngleAxisd(M_PI/2, Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(theta, Eigen::Vector3d::UnitZ());

     moveGripper(mv, "r1", Vector3d(0, -0.7, 1.5),  q0 ) ;
   //  moveGripper(mv, "r1", Vector3d(0.1, -0.7, 1.2), q1 ) ;



    trajectory_msgs::JointTrajectory traj ;
    planArmToPoseWithRollConstraint("r1", Vector3d(0.1, -0.7, 1.2), q1, traj) ;

    mv.execTrajectory(traj) ;

    return 0;

    moveArmHoriz(mv, "r2", -0.5, -0.4, 0.9) ;

    addConeToCollisionModel("r1", 0.6, 0.2) ;

    moveArmHoriz(mv, "r2", -0.5, -1.25, 0.9) ;

    resetCollisionModel() ;

    moveArmHoriz(mv, "r2", -0.5, -0.6, 0.9) ;
}


