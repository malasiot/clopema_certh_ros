#include "robot_helpers/Geometry.h"
#include "robot_helpers/Planner.h"
#include <vector>

#include <boost/random/mersenne_twister.hpp>
#include <boost/random/uniform_real.hpp>

using namespace std ;
using namespace Eigen ;

namespace robot_helpers {


class GraspHangingGoalRegion: public GoalRegion {
public:
    GraspHangingGoalRegion(const string &arm_, const Affine3d &orig_, const Vector3d &p_, const Vector3d &dir_):
        orig(orig_), p(p_), dir(dir_), armName(arm_), gen(time(0))
    {

    }

    void sample(std::vector<double> &xyz_rpy) ;


    const Affine3d &orig ;
    Vector3d p, dir ;
    string armName ;

    boost::mt19937 gen;

};


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


void GraspHangingGoalRegion::sample(std::vector<double> &xyz_rpy)
{
    // compute the approach pose so that the gripper is perpendicular to the given direction with a random slant with
    // respect to the horizontal plane

    Vector3d nz = -dir, na, nb ;
    nz.normalize() ;

    double q = sqrt(nz.x() * nz.x() + nz.y() * nz.y()) ;

    if ( q < 1.0e-4 )
    {
        na = Vector3d(0, 1, 0) ;
        nb = nz.cross(na) ;
    }
    else {

        na = Vector3d(-nz.y()/q, nz.x()/q, 0) ;
        nb = Vector3d(-nz.x() * nz.z()/q, -nz.y() * nz.z()/q, q) ;
    }

    Matrix3d r ;
    r << nz, nb, -na ;

    double angle = boost::uniform_real<double>(-M_PI/4, 0)(gen) ;

    double pa = boost::uniform_real<double>(-M_PI/30, M_PI/30)(gen) ;

    double roll = boost::uniform_real<double>(-M_PI/6, M_PI/6)(gen) ;
pa =0 ;
    r =  Quaterniond(r) * AngleAxisd(angle, Eigen::Vector3d::UnitX()) * /* AngleAxisd(pa, Eigen::Vector3d::UnitY()) * */ AngleAxisd(-roll, Eigen::Vector3d::UnitY())  ;

    // we allow the holding arm to move within a box with dimensions t x t x t around the current position

    const double t = 0.5 ;

    boost::uniform_real<double> dist(-t/2.0, t/2.0) ;

    Vector3d pos1_offset = Vector3d(dist(gen), dist(gen), dist(gen)) ;
    //Vector3d pos1_offset(0, 0, 0) ;

    // compute the pose of the holding arm (arm 1)

    Vector3d pos1 = pos1_offset + orig.translation() ;

    double roll1, pitch1, yaw1, x1, y1, z1 ;

    x1 = pos1.x() ; y1 = pos1.y() ; z1 = pos1.z() ;
    rpyFromQuat(Quaterniond(orig.rotation()) * AngleAxisd(roll, Eigen::Vector3d::UnitZ()), roll1, pitch1, yaw1) ;

    // compute the pose of the approach arm (arm 2)

    Vector3d offset = - r * Vector3d(0, 0, 1) * 0.05 ; // offset with respect to the target point

    Vector3d pos2 = offset + p + pos1_offset ;

    Matrix3d ri = r.inverse() ;

    Quaterniond q2(r) ;
    q2.normalize() ;

    double roll2, pitch2, yaw2, x2, y2, z2 ;

    x2 = pos2.x() ; y2 = pos2.y() ; z2 = pos2.z() ;
    rpyFromQuat(q2, roll2, pitch2, yaw2) ;

    if ( armName == "r1" )
    {
        xyz_rpy.push_back(x1) ;  xyz_rpy.push_back(y1) ;   xyz_rpy.push_back(z1) ;
        xyz_rpy.push_back(roll1) ;  xyz_rpy.push_back(pitch1) ; xyz_rpy.push_back(yaw1) ;

        xyz_rpy.push_back(x2) ;  xyz_rpy.push_back(y2) ;   xyz_rpy.push_back(z2) ;
        xyz_rpy.push_back(roll2) ;  xyz_rpy.push_back(pitch2) ; xyz_rpy.push_back(yaw2) ;


    }
    else
    {
        xyz_rpy.push_back(x2) ;  xyz_rpy.push_back(y2) ;   xyz_rpy.push_back(z2) ;
        xyz_rpy.push_back(roll2) ;  xyz_rpy.push_back(pitch2) ; xyz_rpy.push_back(yaw2) ;

        xyz_rpy.push_back(x1) ;  xyz_rpy.push_back(y1) ;   xyz_rpy.push_back(z1) ;
        xyz_rpy.push_back(roll1) ;  xyz_rpy.push_back(pitch1) ; xyz_rpy.push_back(yaw1) ;

    }



}

void graspHangingPlanApproach(ros::Publisher &pub, const string &armName, const Vector3d &p, const Vector3d &perp_dir)
{

    KinematicsModel kmodel ;
    kmodel.init() ;

    // setup planner for dual arms

    MA1400_R1_IKSolver solver_r1 ;
    solver_r1.setKinematicModel(&kmodel);

    MA1400_R2_IKSolver solver_r2 ;
    solver_r2.setKinematicModel(&kmodel);

    boost::shared_ptr<PlanningContext> pctx(new PlanningContextDual("arms", &kmodel, &solver_r1, "r1_ee", &solver_r2, "r2_ee" ) ) ;

    JointSpacePlanner planner(pctx) ;

    // get pose of the end effector for the arm tip holding the cloth

    KinematicsModel *model = pctx->getModel() ;

    Affine3d orig_pose = model->getWorldTransform(armName + "_ee") ;

    Vector3d rp = orig_pose.inverse() * p ;
    Vector3d rdir = orig_pose.rotation().inverse() * perp_dir ;

    GraspHangingGoalRegion rg(armName, orig_pose, p, perp_dir) ;

    JointTrajectory traj ;

    bool rplan = planner.solve(rg, traj) ;

 //   traj.completeTrajectory(kmodel.getJointState()) ;

    trajectory_msgs::JointTrajectory msg = traj.toMsg(10), filtered ;

    filterTrajectory("arms", msg, filtered) ;

    MoveRobot mv ;

    mv.execTrajectory(filtered) ;

    JointState js(traj.names, traj.positions.back()) ;
    kmodel.setJointState(js) ;

    Affine3d final_pose = model->getWorldTransform(armName + "_ee") ;

    Vector3d fp = final_pose * rp ;
    Vector3d fdir = final_pose.rotation() * rdir ;

    publishTargetMarker(pub, fp, fdir);


}


}
