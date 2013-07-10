#include "ros/ros.h"


#include <robot_helpers/Robot.h>
#include <robot_helpers/Utils.h>

using namespace robot_helpers ;
using namespace std;

bool moveArmHoriz(MoveRobot &cmove, const std::string &armName, double X, double Y, double Z, double theta = 0.0)
{


    Eigen::Quaterniond q ;

    if ( armName == "r2" )
        q = Eigen::AngleAxisd(theta, Eigen::Vector3d::UnitX()) * Eigen::AngleAxisd(-M_PI/2, Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(-M_PI/2, Eigen::Vector3d::UnitZ());
    else
        q = Eigen::AngleAxisd(theta, Eigen::Vector3d::UnitX()) * Eigen::AngleAxisd(M_PI/2, Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(M_PI/2, Eigen::Vector3d::UnitZ());

    return moveGripper(cmove, armName, Eigen::Vector3d(X, Y, Z), q) ;
}


/*
int main(int argc, char *argv[])
{
    ros::init(argc, argv, "qt_ros_test") ;
    ros::NodeHandle nh ;

    MoveRobot mv ;

    moveArmHoriz(mv, "r2", -0.5, -0.4, 0.9) ;

    addConeToCollisionModel("r1", 0.6, 0.2) ;

    moveArmHoriz(mv, "r2", -0.5, -1.25, 0.9) ;

    resetCollisionModel() ;

    moveArmHoriz(mv, "r2", -0.5, -0.6, 0.9) ;




}
*/

void setPathConstraints(clopema_arm_navigation::ClopemaMotionPlan & mp, float radious, const string &armName) {

    string arm2Name = ( armName == "r1 ") ? "r2" : "r1" ;

    mp.request.motion_plan_req.path_constraints.position_constraints.resize(1);
    mp.request.motion_plan_req.path_constraints.position_constraints[0].header.frame_id = armName + "_ee";
    mp.request.motion_plan_req.path_constraints.position_constraints[0].header.stamp = ros::Time::now();
    mp.request.motion_plan_req.path_constraints.position_constraints[0].link_name = armName + "_ee";
    mp.request.motion_plan_req.path_constraints.position_constraints[0].position.x = 0.0;
    mp.request.motion_plan_req.path_constraints.position_constraints[0].position.y = 0.0;
    mp.request.motion_plan_req.path_constraints.position_constraints[0].position.z = 0.0;
//    mp.request.motion_plan_req.path_constraints.position_constraints[0].constraint_region_orientation = G_Orientation;

    mp.request.motion_plan_req.path_constraints.position_constraints[0].constraint_region_shape.type = arm_navigation_msgs::Shape::SPHERE;
    mp.request.motion_plan_req.path_constraints.position_constraints[0].constraint_region_shape.dimensions.push_back(radious); //radius
    mp.request.motion_plan_req.path_constraints.position_constraints[0].constraint_region_shape.dimensions.push_back(radious);
    mp.request.motion_plan_req.path_constraints.position_constraints[0].constraint_region_shape.dimensions.push_back(radious);

    mp.request.motion_plan_req.path_constraints.position_constraints[0].weight = 1.0;

}


int moveWithConstrains(const Eigen::Vector3d &pos, const Eigen::Quaterniond &q, const std::string &armName, float radious) {
        //Create plan

        string arm2Name = ( armName == "r1 ") ? "r2" : "r1" ;

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

        desired_pose.pose.position.x = pos.x() ;
        desired_pose.pose.position.y = pos.y() ;
        desired_pose.pose.position.z = pos.z() ;

        desired_pose.pose.orientation.x = q.x() ;
        desired_pose.pose.orientation.y = q.y() ;
        desired_pose.pose.orientation.z = q.z() ;
        desired_pose.pose.orientation.w = q.w() ;

       // cout<< "\n going to --- >  "<< desired_pose.pose.position.x<< " "  << desired_pose.pose.position.y<<"  " << desired_pose.pose.position.z <<"\n";

        desired_pose.absolute_position_tolerance.x = 0.002;
        desired_pose.absolute_position_tolerance.y = 0.002;
        desired_pose.absolute_position_tolerance.z = 0.002;
        desired_pose.absolute_roll_tolerance = 0.04;
        desired_pose.absolute_pitch_tolerance = 0.04;
        desired_pose.absolute_yaw_tolerance = 0.04;

        setPathConstraints(mp, radious, armName);

        poseToClopemaMotionPlan(mp, desired_pose);

        addSphereToCollisionModel(arm2Name, radious - 0.05);

        ROS_INFO("Planning");
        if (!plan(mp))
            return -1;

        resetCollisionModel() ;

        ROS_INFO("Executing");
        control_msgs::FollowJointTrajectoryGoal goal;
        goal.trajectory = mp.response.joint_trajectory;
        cmove.doGoal(goal);
        return 0;
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "plan_avoiding_cloth") ;
    ros::NodeHandle nh ;

    MoveRobot mv ;

    moveArmHoriz(mv, "r1", -0.4, -0.7, 1.2, M_PI/3) ;


    moveArmHoriz(mv, "r2", -0.4, -0.7, 0.9) ;


    moveWithConstrains(Eigen::Vector3d(-0.1, -0.7, 1.2), Eigen::AngleAxisd(-M_PI/2, Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(-M_PI/2, Eigen::Vector3d::UnitZ()),
                        "r2", 0.3) ;

    moveWithConstrains(Eigen::Vector3d(-0.1, -0.7, 0.9),
                       Eigen::AngleAxisd(M_PI/3, Eigen::Vector3d::UnitX()) * Eigen::AngleAxisd(M_PI/2, Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(M_PI/2, Eigen::Vector3d::UnitZ()),
                        "r1", 0.3) ;





}
