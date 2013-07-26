#include <ros/ros.h>

#include <robot_helpers/KinematicsInterface.h>
#include <robot_helpers/Geometry.h>
#include <robot_helpers/Planner.h>

using namespace std;
using namespace robot_helpers ;
using namespace Eigen ;

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

void planSingle(KinematicsModel &kmodel)
{

    MA1400_R1_IKSolver solver_r1 ;
    solver_r1.setKinematicModel(&kmodel);

    Quaterniond q = lookAt(Eigen::Vector3d(1, 0, 0), M_PI/6) ;

    double roll, pitch, yaw ;
    robot_helpers::rpyFromQuat(q, roll, pitch, yaw) ;

    PlanningContextSingle pctx("r1_arm", &kmodel, &solver_r1, "r1_ee" ) ;

    BoxShapedRegion region(Vector3d(0.2, -0.8, 1.4), Vector3d(0.01, 0.01, 0.01), Vector3d() ) ;

    region.roll_min = -M_PI/4 ;
    region.roll_max = M_PI/4 ;

    region.pitch_min = -M_PI/4 ;
    region.pitch_max = M_PI/4 ;

    JointSpacePlanner planner(&pctx) ;

    JointTrajectory traj ;

    bool rplan = planner.solve(region, traj) ;

    traj.completeTrajectory(kmodel.getJointState()) ;

    trajectory_msgs::JointTrajectory msg = traj.toMsg(10) ;

    MoveRobot mv ;

    mv.execTrajectory(msg) ;

}

void planDual(KinematicsModel &kmodel)
{

    MA1400_R1_IKSolver solver_r1 ;
    solver_r1.setKinematicModel(&kmodel);

    MA1400_R2_IKSolver solver_r2 ;
    solver_r2.setKinematicModel(&kmodel);

    PlanningContextDual pctx("arms", &kmodel, &solver_r1, "r1_ee", &solver_r2, "r2_ee" ) ;

    BoxShapedRegion region1(Vector3d(0.2, -0.8, 1.4), Vector3d(0.05, 0.05, 0.05), Vector3d() ) ;
    BoxShapedRegion region2(Vector3d(0.3, -0.5, 1.4), Vector3d(0.05, 0.05, 0.05), Vector3d() ) ;

    GoalDualCompositeRegion rg(&region1, &region2) ;

    JointSpacePlanner planner(&pctx) ;

    JointTrajectory traj ;

    bool rplan = planner.solve(rg, traj) ;

    traj.completeTrajectory(kmodel.getJointState()) ;

    trajectory_msgs::JointTrajectory msg = traj.toMsg(10) ;

    MoveRobot mv ;

    mv.execTrajectory(msg) ;

}

int main(int argc, char *argv[])
{

    ros::init(argc, argv, "planning") ;

    ros::NodeHandle nh_ ;

    ros::Publisher pub = nh_.advertise<visualization_msgs::MarkerArray>( "visualization_marker_array", 0 );
    ros::Publisher pub2 = nh_.advertise<visualization_msgs::Marker>( "visualization_marker", 0 );

    KinematicsModel kmodel ;
    kmodel.init() ;

    planDual(kmodel) ;

    ros::spin() ;
    Quaterniond q = lookAt(Eigen::Vector3d(1, 0, 0), M_PI/6) ;



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
