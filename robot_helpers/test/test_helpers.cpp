#include <ros/ros.h>

#include <robot_helpers/KinematicsInterface.h>

using namespace std;
using namespace robot_helpers ;

int main(int argc, char *argv[])
{

    ros::init(argc, argv, "planning") ;

    ros::NodeHandle nh_ ;

    ros::Publisher pub = nh_.advertise<visualization_msgs::MarkerArray>( "visualization_marker_array", 0 );

    KinematicsModel kmodel ;
    kmodel.init() ;

    Eigen::Affine3d pose = kmodel.getWorldTransform("r1_ee") ;

    JointState js = JointState::fromRobotState(), solution ;

    MA1400_R1_IKFastSolver solver ;
    solver.setKinematicModel(&kmodel);

    bool res = solver.solveIK("r1_ee", Eigen::Vector3d(0.2, -0.8, 1.4), lookAt(Eigen::Vector3d(1, 0, 0), M_PI/6), js, solution) ;

    MA1400_R1_Xtion_IKFastSolver solver_xtion ;
    solver_xtion.setKinematicModel(&kmodel);

    res = solver_xtion.solveIK("r1_ee", Eigen::Vector3d(0.0, -0.8, 1.4), lookAt(Eigen::Vector3d(1, 0, 0), M_PI), js, solution) ;

    kmodel.setJointState(solution) ;

    visualization_msgs::MarkerArray markers ;
    kmodel.getRobotMarkers(markers);

    cout << kmodel.isStateValid() << endl ;

    while ( ros::ok() )
    {
        pub.publish(markers) ;

        ros::Duration(0.1).sleep() ;
        ros::spinOnce() ;
    }

}
