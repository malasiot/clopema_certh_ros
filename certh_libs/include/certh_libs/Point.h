#ifndef __POINT_H
#define __POINT_H

#include <Eigen/Geometry>
#include <Eigen/StdVector>

namespace certh_libs {

typedef Eigen::Vector2i Point ;
typedef Eigen::Vector2i Vec ;


inline Point max(const Point &a, const Point &b) {
    return Point(std::max(a.x(), b.x()), std::max(a.y(), b.y())) ;
}

inline Point min(const Point &a, const Point &b) {
    return Point(std::min(a.x(), b.x()), std::min(a.y(), b.y())) ;
}

inline Point max(const Point &a, const Point &b, const Point &c) {
    return max(a, max(b, c)) ;
}

inline Point min(const Point &a, const Point &b, const Point &c) {
    return min(a, min(b, c)) ;
}

typedef std::vector<Eigen::Vector2i, Eigen::aligned_allocator<Eigen::Vector2i> > PointList ;

} //namespace cpm ;

#endif

