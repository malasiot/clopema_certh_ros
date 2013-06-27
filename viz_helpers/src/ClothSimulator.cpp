#include <ros/ros.h>

#include <boost/algorithm/string.hpp>

#include "ClothSimulator.h"
#include <planning_environment/monitors/kinematic_model_state_monitor.h>
using namespace std ;


int main(int argc, char *argv[])
{


    ros::init(argc, argv, "camera_viewer") ;
    ros::NodeHandle nh ;

    ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 10);

    planning_environment::CollisionModels cm("robot_description");

    tf::TransformListener listener ;

    planning_environment::KinematicModelStateMonitor monitor(&cm, &listener) ;

    monitor.startStateMonitor() ;
    monitor.waitForState() ;

    planning_models::KinematicState state(monitor.getKinematicModel()) ;
    monitor.setStateValuesFromCurrentValues(state);

    Physics ph(cm, state) ;

    ros::Time prevTime = ros::Time::now(), curTime ;

    Cloth cloth(0.5, 0.7, 20, 20) ;
    ph.addSoftBody("cloth", cloth) ;
    ph.attachSoftBodyToLink("cloth", "corner0", "r1_ee", state);

    while ( ros::ok() )
    {
        // get current state

        monitor.waitForState() ;
        monitor.setStateValuesFromCurrentValues(state);

        // update colision model

        ph.updateCollisions(state) ;

        // update simulation

        curTime = ros::Time::now() ;
        ph.updatePhysicsSimulation((curTime - prevTime).toSec());
        prevTime = curTime ;

        // get marker and publish it
        visualization_msgs::Marker marker ;
        ph.getMeshMarker("cloth", marker);

        marker_pub.publish(marker) ;

        prevTime = curTime ;

        ros::spinOnce() ;
    }


}
