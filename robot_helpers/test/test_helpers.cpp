#include <ros/ros.h>

#include <robot_helpers/KinematicsInterface.h>
#include <robot_helpers/Geometry.h>
#include <robot_helpers/Planner.h>
#include <robot_helpers/Planners.h>
#include <planning_environment/util/construct_object.h>

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

    PlanningContextPtr pctx(new PlanningContextSingle("r1_arm", &kmodel, &solver_r1, "r1_ee" ) );

    BoxShapedRegion *region = new BoxShapedRegion(Vector3d(0.6, -0.7, 1.4), Vector3d(0.01, 0.01, 0.01), Vector3d() ) ;

    region->roll_min = -M_PI/4 ;
    region->roll_max = M_PI/4 ;

    region->pitch_min = -M_PI/4 ;
    region->pitch_max = M_PI/4 ;

    JointSpacePlanner planner(pctx) ;

    JointTrajectory traj ;

    GoalRegionPtr rg(region) ;

    bool rplan = planner.solve(rg, traj) ;

    if ( rplan )
    {

//    traj.completeTrajectory(kmodel.getJointState()) ;

        trajectory_msgs::JointTrajectory msg = traj.toMsg(10), filtered ;

        filterTrajectory("r1_arm", msg, filtered) ;


        MoveRobot mv ;

        mv.execTrajectory(msg) ;
    }

}

void planSingleTaskSpace(KinematicsModel &kmodel)
{

    MA1400_R1_IKSolver solver_r1 ;
    solver_r1.setKinematicModel(&kmodel);

    Quaterniond q = lookAt(Eigen::Vector3d(1, 0, 0), M_PI/6) ;

    double roll, pitch, yaw ;
    robot_helpers::rpyFromQuat(q, roll, pitch, yaw) ;

    PlanningContextPtr pctx(new PlanningContextSingle("r1_arm", &kmodel, &solver_r1, "r1_ee" ) );

    BoxShapedRegion region(Vector3d(0.3, -0.5, 1.4), Vector3d(0.01, 0.01, 0.01), Vector3d() ) ;

    region.roll_min = -M_PI/6 ;
    region.roll_max = M_PI/6;

    region.pitch_min = -M_PI/6 ;
    region.pitch_max = M_PI/6 ;

    RPY_XYZ_TaskSpace ts ;

    TaskSpacePlanner planner(pctx) ;

    JointTrajectory traj ;

    bool rplan = planner.solve(region, ts, traj) ;

  // ros::Duration(10).sleep() ;

//    traj.completeTrajectory(kmodel.getJointState()) ;


    trajectory_msgs::JointTrajectory msg = traj.toMsg(10), filtered ;

    filterTrajectory("r1_arm", msg, filtered) ;

    MoveRobot mv ;

    mv.execTrajectory(filtered) ;

}

bool distanceConstraint(const PlanningContextPtr &ctx, double dist, const JointState &js)
{

    KinematicsModel *model = ctx->getModel() ;

    model->setJointState(js) ;

    Affine3d p1 = model->getWorldTransform("r1_ee") ;
    Affine3d p2 = model->getWorldTransform("r2_ee") ;

    double d = (p1.translation() - p2.translation()).norm() ;


    return d <= dist ;
}

void planDual(KinematicsModel &kmodel)
{

    boost::shared_ptr<PlanningContext> pctx(new PlanningContextDualDefault( &kmodel ) );

    GoalRegionPtr region1(new BoxShapedRegion(Vector3d(0.0, -0.8, 1.4), Vector3d(0.1, 0.1, 0.1), Vector3d() )) ;
    GoalRegionPtr region2(new BoxShapedRegion(Vector3d(0.1, -0.5, 1.4), Vector3d(0.1, 0.1, 0.1), Vector3d() )) ;

    GoalRegionPtr rg(new GoalDualCompositeRegion(region1, region2)) ;

    JointSpacePlanner planner(pctx) ;

    planner.addStateValidityChecker(boost::bind(distanceConstraint, pctx, 0.4, _1));

    JointTrajectory traj ;

    bool rplan = planner.solve(rg, traj) ;

 //   traj.completeTrajectory(kmodel.getJointState()) ;

    trajectory_msgs::JointTrajectory msg = traj.toMsg(10), filtered ;

    filterTrajectory("arms", msg, filtered) ;

    MoveRobot mv ;

    mv.execTrajectory(filtered) ;

}


void publishTargetMarker(ros::Publisher &vis_pub, const Eigen::Vector3d &p, const Eigen::Vector3d &n)
{


    visualization_msgs::Marker marker;
        // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = "base_link";
    marker.header.stamp = ros::Time::now();

    marker.ns = "target point";
    marker.id = 0;

    // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
    marker.type = visualization_msgs::Marker::ARROW ;

    Eigen::Vector3d ep = p + 0.3 *n ;

    geometry_msgs::Point p1, p2 ;
    p1.x = p.x() ;
    p1.y = p.y() ;
    p1.z = p.z() ;
    p2.x = ep.x() ;
    p2.y = ep.y() ;
    p2.z = ep.z() ;

    marker.points.push_back(p1) ;
    marker.points.push_back(p2) ;

    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;

    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;

    marker.lifetime = ros::Duration();

    vis_pub.publish(marker);

}

bool attachConeToCollisionModel(arm_navigation_msgs::AttachedCollisionObject &att_object, const std::string &armName, double length, double radius)
{
    att_object.link_name = armName + "_gripper";

    att_object.object.id = "attached_cone";
    att_object.object.operation.operation = arm_navigation_msgs::CollisionObjectOperation::ADD;

    att_object.object.header.frame_id = "base_link";
    att_object.object.header.stamp = ros::Time::now();

    Eigen::Vector3d p = robot_helpers::getPose(armName).inverse().translation() ;


    arm_navigation_msgs::Shape object;

    shapes::Mesh mesh ;
    makeSolidCone(mesh, radius, length, 10, 20) ;

    if(!planning_environment::constructObjectMsg(&mesh, object)) {
      ROS_WARN_STREAM("Object construction fails");
    }

    geometry_msgs::Pose pose;
    pose.position.x = p.x();
    pose.position.y = p.y();
    pose.position.z = p.z() ;
    pose.orientation.x = 0;
    pose.orientation.y = 0;
    pose.orientation.z = 0;
    pose.orientation.w = 1;

    att_object.object.shapes.push_back(object);
    att_object.object.poses.push_back(pose);

    return true ;
}

int main(int argc, char *argv[])
{

    ros::init(argc, argv, "planning") ;

    ros::NodeHandle nh_ ;


    ros::AsyncSpinner spinner(4) ;
    spinner.start() ;

    ros::Publisher pub = nh_.advertise<visualization_msgs::MarkerArray>( "visualization_marker_array", 0 );
    ros::Publisher pub2 = nh_.advertise<visualization_msgs::Marker>( "visualization_marker", 0 );






        KinematicsModel kmodel ;
        kmodel.init() ;

          MoveRobot mv ;
    moveHome(mv) ;
    moveGripperPointingDown(mv, "r1", 0, -0.7, 1.6) ;

    moveGripper(mv, "r2", Vector3d(0, -0.75, 0.8), lookAt(Vector3d(-1, 0, 0))) ;
/*
    Vector3d pos(-0.1, -0.8, 1.0 ) ;
    Vector3d dir(1, 0, 0) ;

    GraspHangingPlanner gsp(kmodel, "r1") ;
    trajectory_msgs::JointTrajectory traj ;

    if ( gsp.plan(pos, dir, traj) )
    {
        cout << "ok here" << endl ;
        mv.execTrajectory(traj) ;

        publishTargetMarker(pub2, gsp.fp, gsp.fdir);
    }
*/

    FlipHandsPlanner fh(kmodel, "r1") ;

    trajectory_msgs::JointTrajectory traj ;

    if ( fh.plan(traj) )
    {
        cout << "ok here" << endl ;
        mv.execTrajectory(traj) ;

    }





//    planDual(kmodel) ;


    ros::spin() ;


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
