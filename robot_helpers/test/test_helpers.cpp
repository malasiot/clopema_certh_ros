#include <ros/ros.h>

#include <robot_helpers/PlanningContext.h>

using namespace std;
using namespace robot_helpers ;

int main(int argc, char *argv[])
{


    ros::init(argc, argv, "planning") ;

    ros::NodeHandle nh_ ;

    ros::Publisher pub = nh_.advertise<visualization_msgs::MarkerArray>( "visualization_marker_array", 0 );


    PlanningContext ctx ;
    ctx.init("r1_arm") ;

    ctx.initFromRobotState() ;

    sensor_msgs::JointState state ;

    arm_navigation_msgs::RobotState rs ;
    getRobotState(rs) ;

    ctx.solveIK("r1_ee", Eigen::Vector3d(0, -0.8, 0.9), lookAt(Eigen::Vector3d(1, 0, 0), M_PI), state) ;

    //getIK("r1", Eigen::Vector3d(0.2, -0.8, 0.6), lookAt(Eigen::Vector3d(0, 1, 0)), state) ;

    ctx.setStateFromJointState(state) ;

    visualization_msgs::MarkerArray markers ;
    ctx.getRobotMarkers(markers);

    cout << ctx.isStateValid() << endl ;

    while ( ros::ok() )
    {
        pub.publish(markers) ;

        ros::Duration(0.1).sleep() ;
        ros::spinOnce() ;
    }


    cout << "ok" << endl ;
}
