#include <robot_helpers/Geometry.h>
#include <iostream>
#include <Eigen/Geometry>

using namespace std ;
using namespace Eigen ;

namespace robot_helpers {

ostream &operator << (ostream &strm, const Quaterniond &q)
{
    cout << "[ " << q.x() << ' ' << q.y() << ' ' << q.z() << ' ' << q.w() << " ]";

    return strm ;
}

Quaterniond quatFromRPY(double roll, double pitch, double yaw)
{
    Quaterniond q ;

    q = AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());

    return q ;
}

void rpyFromQuat(const Quaterniond &q, double &roll, double &pitch, double &yaw)
{
    Matrix3d r = q.toRotationMatrix() ;
    Vector3d euler = r.eulerAngles(2, 1, 0) ;
    yaw = euler.x() ;
    pitch = euler.y() ;
    roll = euler.z() ;
}

Quaterniond lookAt(const Eigen::Vector3d &dir, double roll)
{
     Vector3d nz = dir, na, nb ;
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
     r << na, nb, nz ;

     return Quaterniond(r) * AngleAxisd(roll, Eigen::Vector3d::UnitZ()) ;
}

} // namespace robot_helpers
