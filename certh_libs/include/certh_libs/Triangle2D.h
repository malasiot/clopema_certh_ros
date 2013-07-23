#ifndef __TRIANGLE_2D_H
#define __TRIANGLE_2D_H

#include <certh_libs/Point2D.h>
#include <certh_libs/Rectangle2D.h>

namespace certh_libs {

class Triangle2D
{
public:

    Triangle2D() {}
    Triangle2D(const Point2D &A, const Point2D &B, const Point2D &C):
        p1(A), p2(B), p3(C) {}
    Triangle2D(const Triangle2D &other):
        p1(other.p1), p2(other.p2), p3(other.p3) {}
    Triangle2D(const Point2D pl[3]):
        p1(pl[0]), p2(pl[1]), p3(pl[2]) {}

    float area() const ;
    Rectangle2D boundingBox() const ;

    bool contains(const Point2D &p) const ;
    bool contains(double x, double y) const { return contains(Point2D(x,y)) ; }

    friend std::ostream &operator << (std::ostream &strm, const Triangle2D &p) ;

    enum ICode { PtInTriangle=1, PtOnEdge=0, PtOnVertex=-1, PtOutTriangle=-2 } ;

    ICode whereIs(const Point2D &p) const ;

    void getBarycentric(double x, double y, double &g0, double &g1, double &g2) const ;

    Point2D p1, p2, p3 ;

    static int orientation(const Point2D &A, const Point2D &B, const Point2D &C) ;
} ;




}

#endif
