#ifndef _POINT_LIST_2D_H_
#define _POINT_LIST_2D_H_

#include <vl.h>
#include <Point2D.h>
#include <Transform2D.h>
#include <vector>

namespace vl {

class PointList2Ds ;

class DLLAPI PointList2D: public std::vector<Point2D>
{
  public:

  PointList2D() {} ;
  PointList2D(double *x, double *y, int n) ;
  PointList2D(const PointList2D &other) ;
  PointList2D(const std::vector<Point2D> &pts) ;
  //PointList2D(const Vector &x) ;

  Point2D Center() const ;
  void Axes(double &l1, Vector2 &v1, double &l2, Vector2 &v2) const ;

  // Procrustes analysis 
  // Find the transform  that aligns this shape with the other one:
  // this' = T(s) * T(theta) * this + T(tx, ty)  
  void Align(const PointList2D &other, double &theta, double &scale, double &tx, double &ty) const ;

  void Align(const PointList2D &other, double *weights, double &theta, double &scale, double &tx, double &ty) const ;

  void Transform(const AffineTransform &xf) ;
  void BBox(Point2D &ul, Point2D &br) const ;

 // Vector GetData() const ;
 // void SetData(const Vector &);

  //friend DLLAPI Archive &operator << (Archive &, const PointList2D &v) ;
  //friend DLLAPI Archive &operator >> (Archive &, PointList2D &v) ;

  void Copy(const PointList2Ds &v); 
} ;

///////////////////////////////////////////////////////////////////////////////////////////////////////

class DLLAPI PointList2Ds: public std::vector<Point2Ds>
{
  public:

  PointList2Ds() {} ;
  PointList2Ds(float *x, float *y, int n) ;
  PointList2Ds(const PointList2Ds &other) ;
  PointList2Ds(const std::vector<Point2Ds> &pts) ;
 // PointList2Ds(const VectorS &x) ;

  Point2Ds Center() const ;
  void Axes(float &l1, Vector2s &v1, float &l2, Vector2s &v2) const ;

  // Procrustes analysis 
  // Find the transform  that aligns this shape with the other one:
  // this' = T(s) * T(theta) * this + T(tx, ty)  
  void Align(const PointList2Ds &other, float &theta, float &scale, float &tx, float &ty) const ;

  void Align(const PointList2Ds &other, float *weights, float &theta, float &scale, float &tx, float &ty) const ;

  void Transform(const AffineTransformS &xf) ;
  void BBox(Point2Ds &ul, Point2Ds &br) const ;

  //VectorS GetData() const ;
  //void SetData(const VectorS &);

  //friend DLLAPI Archive &operator << (Archive &, const PointList2Ds &v) ;
  //friend DLLAPI Archive &operator >> (Archive &, PointList2D &v) ;
  
  void Copy(const PointList2D &v);  

} ;

} // namespace cpm

#endif
