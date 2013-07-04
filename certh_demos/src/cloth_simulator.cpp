#include <ros/ros.h>
#include <robot_helpers/Utils.h>
#include <viz_helpers/ClothSimulatorService.h>

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "cloth_simulator_test") ;
    ros::NodeHandle nh ;

    robot_helpers::MoveRobot rb ;

    Eigen::Quaterniond q ;
     q = Eigen::AngleAxisd(-M_PI/2, Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(-M_PI/2, Eigen::Vector3d::UnitZ());

    if ( !robot_helpers::setRobotSpeed(0.01) )
        exit(1) ;
    robot_helpers::moveGripper(rb, "r2", Eigen::Vector3d(0, -0.9, 1.4), q) ;

    // access to cloth simulator service
    ros::service::waitForService("cloth_simulator_service") ;

    viz_helpers::ClothSimulatorService::Request req ;
    viz_helpers::ClothSimulatorService::Response res ;

    // add a rectangular cloth

    req.action = viz_helpers::ClothSimulatorService::Request::ADD ;
    req.name = "towel" ;
    req.type = viz_helpers::ClothSimulatorService::Request::TOWEL ;

    req.width = 0.5 ;
    req.height = 0.7 ;
    req.nodes = 250 ;   // number of nodes of the mesh
    req.anchors.push_back("corner0") ; // This is a list of points on the soft body that will be attached to corresponding rigid links
                                       // For the towel these are "corner0", "corner1", "corner2", "corner3", "#node-idx" (an arbitrary mesh node)
    req.links.push_back("r2_ee") ;     // This is the link to which we attach "corner0"
    req.pose.position.x = 0 ;           // The following is the initial position and orientation of the cloth before the simulation starts
    req.pose.position.y = -1.0 ;
    req.pose.position.z = 1.5 ;
    req.pose.orientation.x = 0 ;
    req.pose.orientation.y = 0 ;
    req.pose.orientation.z = 0 ;
    req.pose.orientation.w = 1 ;

    ros::service::call("cloth_simulator_service", req, res) ;

    ros::Duration(5).sleep() ;

    robot_helpers::moveGripper(rb, "r2", Eigen::Vector3d(0, -0.38, 1.0), q) ;

    // remove the item from the simulation

    req.action = viz_helpers::ClothSimulatorService::Request::REMOVE ;

    ros::service::call("cloth_simulator_service", req, res) ;

    ros::Duration(5).sleep() ;
}
