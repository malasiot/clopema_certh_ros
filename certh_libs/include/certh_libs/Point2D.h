#ifndef __POINT_2D_H
#define __POINT_2D_H

#include <Eigen/Geometry>
#include <Eigen/StdVector>

namespace certh_libs {

typedef Eigen::Vector2d Vector2 ;
typedef Eigen::Vector2d Point2D ;
typedef Eigen::Vector2d Vec2D ;


inline Point2D max(const Point2D &a, const Point2D &b) {
    return Point2D(std::max(a.x(), b.x()), std::max(a.y(), b.y())) ;
}

inline Point2D min(const Point2D &a, const Point2D &b) {
    return Point2D(std::min(a.x(), b.x()), std::min(a.y(), b.y())) ;
}

inline Point2D max(const Point2D &a, const Point2D &b, const Point2D &c) {
    return max(a, max(b, c)) ;
}

inline Point2D min(const Point2D &a, const Point2D &b, const Point2D &c) {
    return min(a, min(b, c)) ;
}

typedef std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d> > PointList2D ;

} // namespace certh_libs

#endif

