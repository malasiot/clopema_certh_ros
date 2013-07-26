#include <robot_helpers/TaskSpace.h>
#include <robot_helpers/Geometry.h>
#include <Eigen/Geometry>

using namespace std ;
using namespace Eigen ;

namespace robot_helpers {

extern void rpy_to_quat(double r, double p, double y, double &qx, double &qy, double &qz, double &qw) ;

RPY_XYZ_TaskSpace::RPY_XYZ_TaskSpace(): TaskSpace()
{
    upper[0] = 2 ; lower[0] = -2 ;
    upper[1] = 2 ; lower[1] = -2 ;
    upper[2] = 2 ; lower[2] = -2 ;

    upper[3] = M_PI ; lower[3] = -M_PI ;
    upper[4] = M_PI ; lower[4] = -M_PI ;
    upper[5] = M_PI ; lower[5] = -M_PI ;
}

int RPY_XYZ_TaskSpace::getDimension() const  { return 6 ; }

double RPY_XYZ_TaskSpace::getUpperLimit(int dim) const { return upper[dim] ; }
double RPY_XYZ_TaskSpace::getLowerLimit(int dim) const { return lower[dim] ; }

void RPY_XYZ_TaskSpace::taskSpaceToPose(const std::vector<double> &state, vector<Affine3d> &pose) const
{
    double x = state[0] ;
    double y = state[1] ;
    double z = state[2] ;

    double roll = state[3] ;
    double pitch = state[4] ;
    double yaw = state[5] ;

    double qx, qy, qz, qw ;

    rpy_to_quat(roll, pitch, yaw, qx, qy, qz, qw) ;

    Quaterniond rot(qw, qx, qy, qz) ;
    Translation3d trans(x, y, z) ;

    pose.push_back(trans * rot.toRotationMatrix()) ;
}

void poseToXYZRPY(const Affine3d &pose, double &X, double &Y, double &Z, double &roll, double &pitch, double &yaw) ;

// transform pose of the end-effector to state in task space
void RPY_XYZ_TaskSpace::poseToTaskSpace(const vector<Affine3d> &pose, std::vector<double> &state) const
{
    double X, Y, Z, roll, pitch, yaw ;

    poseToXYZRPY(pose[0], X, Y, Z, roll, pitch, yaw) ;

    state.push_back(X) ;
    state.push_back(Y) ;
    state.push_back(Z) ;

    state.push_back(roll) ;
    state.push_back(pitch) ;
    state.push_back(yaw) ;
}



/////////////////////////////////////////////////////////////////////////////////////////////////////////

Matrix3d getLookAtMatrix(const Vector3d &n)
{
    return lookAt(n).toRotationMatrix() ;
}


MoveTo_TaskSpace::MoveTo_TaskSpace(const Affine3d &pose, const Vector3d &dp, double pos_tol,
                     const Vector3d &rpy_minus, const Vector3d &rpy_plus)
{
    double X, Y, Z, roll, pitch, yaw ;

    poseToXYZRPY(pose, X, Y, Z, roll, pitch, yaw) ;

    c0 = Vector3d(X, Y, Z) ;
    a0 = Vector3d(roll, pitch, yaw) ;

    Vector3d c1 = c0 + dp ;

    // We define a coordinate system with the Z axis pointing towards the target point

    Matrix3d r = getLookAtMatrix(dp) ;

    frame = r  ;
    iframe = r.inverse() ;

    // we use a cylinder parameterization of the position

    lower[0] = 0.0      ; upper[0] = dp.norm() ; // cylinder length
    lower[1] = -M_PI    ; upper[1] = M_PI ; // polar angle
    lower[2] = 0.0      ; upper[2] = pos_tol ; // radius

    const double small_ = 0.001 ;

    double roll_min = std::max(a0.x() - fabs(rpy_minus.x()) - small_, -M_PI) ;
    double roll_max = std::min(a0.x() + fabs(rpy_plus.x()) + small_, M_PI) ;

    double pitch_min = std::max(a0.y() - fabs(rpy_minus.y()) - small_, -M_PI) ;
    double pitch_max = std::min(a0.y() + fabs(rpy_plus.y()) + small_, M_PI) ;

    double yaw_min = std::max(a0.z() - fabs(rpy_minus.z()) - small_, -M_PI) ;
    double yaw_max = std::min(a0.z() + fabs(rpy_plus.z()) + small_, M_PI) ;

    upper[3] = roll_max     ; lower[3] = roll_min ;
    upper[4] = pitch_max    ; lower[4] = pitch_min ;
    upper[5] = yaw_max      ; lower[5] = yaw_min ;

}



void MoveTo_TaskSpace::taskSpaceToPose(const std::vector<double> &state, vector<Affine3d> &pose) const
{
    double l = state[0] ;
    double theta = state[1] ;
    double r = state[2] ;

    double x = r * cos(theta) ;
    double y = r * sin(theta) ;
    double z = l  ;

    Vector3d pt = frame * Vector3d(x, y, z)  + c0 ;

    double roll = state[3] ;
    double pitch = state[4] ;
    double yaw = state[5] ;

    double qx, qy, qz, qw ;

    rpy_to_quat(roll, pitch, yaw, qx, qy, qz, qw) ;

    Quaterniond rot(qw, qx, qy, qz) ;
    Translation3d trans(pt) ;

    pose.push_back( trans * rot.toRotationMatrix() ) ;

}

void MoveTo_TaskSpace::poseToTaskSpace(const vector<Affine3d> &pose, std::vector<double> &state) const
{
    double X, Y, Z, roll, pitch, yaw ;

    poseToXYZRPY(pose[0], X, Y, Z, roll, pitch, yaw) ;

    Vector3d pt = iframe * ( Vector3d(X, Y, Z) - c0 )  ;

    double l = pt.z()  ;
    double t = atan2(pt.y(), pt.x()) ;
    double r = sqrt(pt.x() * pt.x() + pt.y() * pt.y()) ;

    state.push_back(l) ;
    state.push_back(t) ;
    state.push_back(r) ;

    state.push_back(roll) ;
    state.push_back(pitch) ;
    state.push_back(yaw) ;

}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void TaskSpaceDualComposite::taskSpaceToPose(const std::vector<double> &state, std::vector<Affine3d> &pose) const
{
    int d1 = lspace->getDimension(), d2 = rspace->getDimension() ;

    vector<double> state1, state2 ;
    vector<Affine3d> pose_l, pose_r ;

    state1.insert(state1.end(), state.begin(), state.begin() + d1);
    state2.insert(state2.end(), state.begin() + d1, state.end());

    lspace->taskSpaceToPose(state1, pose_l) ;
    rspace->taskSpaceToPose(state2, pose_r) ;

    pose.push_back(pose_l[0]) ;
    pose.push_back(pose_r[0]) ;

}

void TaskSpaceDualComposite::poseToTaskSpace(const vector<Affine3d> &pose, std::vector<double> &state) const
{
    vector<double> state1, state2 ;

    std::vector<Affine3d> pose_l, pose_r ;

    pose_l.push_back(pose[0]) ;
    pose_r.push_back(pose[1]) ;
    lspace->poseToTaskSpace(pose_l, state1) ;
    rspace->poseToTaskSpace(pose_r, state2) ;

    state.insert(state.end(), state1.begin(), state1.end()) ;
    state.insert(state.end(), state2.begin(), state2.end()) ;

}

}
