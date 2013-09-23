#ifndef __CERTH_LIBS_GEOMETRY_H__
#define __CERTH_LIBS_GEOMETRY_H__

#include <cv.h>

#include <Eigen/Core>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>


typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef Eigen::Vector3d Vec3 ;
typedef Eigen::Matrix3d Matrix3 ;

namespace certh_libs {

// test if given point lies within cone emanating from apex to the base and with given apperture angle

bool pointInsideCone(const Vec3 &x, const Vec3 &apex, const Vec3 &base, float aperture) ;

// robust fit plane to a list of 3D points u'( p - c ) = 0 ;

double robustPlane3DFit(std::vector<Vec3> &x, Vec3 &c, Vec3 &u) ;

// compute normal vector from a point cloud (depth map) at given coordinates

Vec3 computeNormal(const PointCloud &pc, int x, int y) ;

} // namespace certh_libs

#endif
