#include <ros/ros.h>

#include <robot_helpers/KinematicsInterface.h>
#include <robot_helpers/Geometry.h>

using namespace std;
using namespace robot_helpers ;

void createMarkerFromMesh( shapes::Mesh &mesh, const Eigen::Affine3d &trans, visualization_msgs::Marker &marker)
{


    marker.header.frame_id = "/base_link";
    marker.header.stamp = ros::Time::now();

    marker.type = visualization_msgs::Marker::TRIANGLE_LIST ;

    int k=0 ;

    for( int i=0 ; i<mesh.triangleCount; i++ )
    {
        int v1 = mesh.triangles[k++] ;
        int v2 = mesh.triangles[k++] ;
        int v3 = mesh.triangles[k++] ;

        geometry_msgs::Point p ;

        p.x = mesh.vertices[v1*3] ;
        p.y = mesh.vertices[v1*3+1] ;
        p.z = mesh.vertices[v1*3+2] ;

        marker.points.push_back(p) ;

        p.x = mesh.vertices[v2*3] ;
        p.y = mesh.vertices[v2*3+1] ;
        p.z = mesh.vertices[v2*3+2] ;

        marker.points.push_back(p) ;

        p.x = mesh.vertices[v3*3] ;
        p.y = mesh.vertices[v3*3+1] ;
        p.z = mesh.vertices[v3*3+2] ;

        marker.points.push_back(p) ;

    }

    marker.color.r = 1.0f;
    marker.color.g = 0.5f;
    marker.color.b = 0.7f;
    marker.color.a = 0.5;

    marker.pose = eigenPoseToROS(trans.translation(), Eigen::Quaterniond(trans.rotation())) ;


    marker.scale.x = 1.0 ;
    marker.scale.y = 1.0 ;
    marker.scale.z = 1.0 ;

    marker.lifetime = ros::Duration();

}

int main(int argc, char *argv[])
{

    ros::init(argc, argv, "planning") ;

    ros::NodeHandle nh_ ;

    ros::Publisher pub = nh_.advertise<visualization_msgs::MarkerArray>( "visualization_marker_array", 0 );
    ros::Publisher pub2 = nh_.advertise<visualization_msgs::Marker>( "visualization_marker", 0 );

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

 //   kmodel.setJointState(solution) ;

    visualization_msgs::MarkerArray markers ;
    kmodel.getRobotMarkers(markers);

    cout << kmodel.isStateValid() << endl ;

    shapes::Mesh mesh ;

    makeCameraFrustum(mesh, 0.6, 2.0, 58*M_PI/180, 45*M_PI/180, false, true);

    visualization_msgs::Marker marker ;

    createMarkerFromMesh(mesh, kmodel.getWorldTransform("r1_xtion"), marker) ;
    marker.ns = "xtion r1 frustum" ;
    marker.id = 0 ;

    while ( ros::ok() )
    {
    //    pub.publish(markers) ;
        pub2.publish(marker) ;

        ros::Duration(0.1).sleep() ;
        ros::spinOnce() ;
    }

}
