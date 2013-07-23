#ifndef _LINE2D_H_
#define _LINE2D_H_

#include <certh_libs/Point2D.h>

#include <vector>

namespace certh_libs {

// 2D line 

class Line2D
{
  public:

  friend class LineSegment2D ;
  // Constructors

  // Line passing from two points
  Line2D(const Point2D &p1, const Point2D &p2) ;

  // Line passing from a point p with direction dir
//  Line2D(const Point2D &p, const Vec2D &dir) ;

  // Line in the form a x + b * y + c = 0 ;
  Line2D(double a, double b, double c) ;

  // Line in the form a[0] * x + a[1] * y + a[2] = 0 ;

  Line2D(double a[3]) ;

  // 2D line regression. if robust = true applies robust fitting algorithm

  Line2D(const std::vector<Point2D> &pts, bool robust = false ) ;

  // Get line direction vector

  Vec2D GetDir() const { return d ; }

  void GetCoefs(double &a, double &b, double &c) const ;
  void GetCoefs(double a[3]) const ;

  // Find the intersection between  two lines

  bool Intersection(const Line2D &other, Point2D &p) const ;

  bool IsParallel(const Line2D &other) const ;

  // Find the distance of a point to the line. Optionally returns the closest point on the line
  double DistanceToPoint(const Point2D &p, Point2D *psd = NULL) const ;

 // friend CPMDLLAPI Archive &operator << (Archive &a, const Line2D &l) ;
  //friend CPMDLLAPI Archive &operator >> (Archive &a, Line2D &l) ;
  
  Point2D p ;
  Vec2D   d ;

    private:

  void init(double, double, double) ;
} ;

// 2D line segment

class LineSegment2D
{
  public:

  // Constructors

  // Line segment defined from  two points
  LineSegment2D(const Point2D &p1, const Point2D &p2) ;

  // Line segment starting from points p and p + d
//  LineSegment2D(const Point2D &p, const Vec2D &dir) ;

  // Line regression. End points are determined by projecting all points on the
  // the line

  LineSegment2D(const std::vector<Point2D> &pts, bool robust = true ) ;

  Line2D GetLine() const ;

  // Get line direction vector

  Vector2 GetDir() const ;

  // Swap end-points

  void Invert() ;

  // Find the intersection between the segment and a line

  bool Intersection(const Line2D &other, Point2D &p) const ;

  // Find the intersection (if any) with a line segment

  bool Intersection(const LineSegment2D &other, Point2D &p) const ;

  bool IsParallel(const Line2D &other) const ;
  bool IsParallel(const LineSegment2D &other) const  ;

  bool IsColinear(const Line2D &other) const ;
  bool IsColinear(const LineSegment2D &other) const ;

  // Find the distance of a point to the line. 
  // Optionally returns the closest point on the line
  
  double DistanceToPoint(const Point2D &p, Point2D *psd = NULL) const ;

  // For points that lye on the line the function returns a flag indicating 
  // whether the point is inside the line segment (0), before the first end-point
  // (-1) and after the second end-point (1)

  int Contains(const Point2D &p) const ;

 // friend CPMDLLAPI Archive &operator << (Archive &a, const LineSegment2D &l) ;
 // friend CPMDLLAPI Archive &operator >> (Archive &a, LineSegment2D &l) ;

  Point2D pa, pb ;
} ;

}

#endif
