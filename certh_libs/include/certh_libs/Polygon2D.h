#ifndef __POLYGON_2D_H__
#define __POLYGON_2D_H__

#include <certh_libs/Triangle2D.h>
#include <certh_libs/Rectangle2D.h>

#include <Eigen/StdDeque>

namespace certh_libs {

class TriangleMesh2D ;

class Polygon2D
{
    public:

    Polygon2D(): plist(), trg(NULL) {}
    Polygon2D(const Point2D *p, int n) ;
    Polygon2D(const Polygon2D &other): plist(other.plist), trg(NULL) {}
    Polygon2D(int n): plist(n), trg(NULL) {}
    Polygon2D(const Triangle2D &tr) ;
    ~Polygon2D() ;

    void clear() { plist.clear() ; }

    int getNumPoints() const { return plist.size() ; }
  
    void addPoint(const Point2D &p) { plist.push_back(p) ; }
    Point2D &getPoint(int i) { return plist[i] ; }
    Point2D &operator [] (int i) { return plist[i] ; }
    const Point2D &operator [] (int i) const
        { return (const Point2D &)plist[i] ; }
  
    double area() const ;
    Rectangle2D boundingBox() const ;

    enum ICode { PtInPolygon, PtOutPolygon, PtOnEdge, PtOnVertex } ;

    ICode whereIs(const Point2D &p) ;

    bool contains(const Point2D &p) ;
    bool contains(float x, float y) { return contains(Point2D(x,y)) ; }

    friend std::ostream &operator << (std::ostream &strm, const Polygon2D &p) ;
    friend std::istream &operator >> (std::istream &strm, Polygon2D &p) ;

private:

    std::deque<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d> > plist ;

    TriangleMesh2D *trg ;
} ;


inline Polygon2D::Polygon2D(const Triangle2D &tr): plist(), trg(NULL)
{
    addPoint(tr.p1) ;
    addPoint(tr.p2) ;
    addPoint(tr.p3) ;
}


}

#endif
