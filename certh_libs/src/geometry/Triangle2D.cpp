#include <certh_libs/Triangle2D.h>

using namespace std ;

namespace certh_libs {

float Triangle2D::area() const
{
    Vector2 v1 = p1 - p2 ;
    Vector2 v2 = p1 - p3 ;
  
    return 0.5*fabs(v1.x() * v2.y() - v1.y() * v2.x()) ;
}
 
Rectangle2D Triangle2D::boundingBox() const
{
    Point2D pmin = min(p1, p2, p3) ;
    Point2D pmax = max(p1, p2, p3) ;

    return Rectangle2D(pmin, pmax) ;
}

ostream &operator << (ostream &strm, const Triangle2D &p) 
{
    strm << p.p1 << ", " << p.p2 << ", " << p.p3 << endl ;

    return strm ;
}

int Triangle2D::orientation(const Point2D &P,
  const Point2D &Q, const Point2D &R)
{
    Vec2D A,B ;
    float d ;

    A = Q - P ;
    B = R - P ;

    d = A.x()*B.y() - A.y()*B.x() ;

    return ( d <= -0.001 ? -1 : d >= 0.001 ) ;
}

Triangle2D::ICode Triangle2D::whereIs(const Point2D &p) const
{
    int orABP,orBCP,orCAP ;

    orABP = orientation(p1,p2,p) ;
    orBCP = orientation(p2,p3,p) ;
    orCAP = orientation(p3,p1,p) ;

    if ( orCAP == orBCP && orABP == orCAP ) return PtInTriangle ;

    if ( orABP == 0 && orCAP == orBCP ) return PtOnEdge ;
    if ( orBCP == 0 && orABP == orCAP ) return PtOnEdge ;
    if ( orCAP == 0 && orBCP == orABP ) return PtOnEdge ;
    if ( orABP == 0 && orCAP == 0 ) return PtOnVertex ;
    if ( orABP == 0 && orBCP == 0 ) return PtOnVertex ;
    if ( orBCP == 0 && orCAP == 0 ) return PtOnVertex ;

    return PtOutTriangle ;
}

bool Triangle2D::contains(const Point2D &p) const
{
    return (whereIs(p) == PtOutTriangle) ? false : true ;
}

void Triangle2D::getBarycentric(double X, double Y, double &g0, double &g1, double &g2) const
{
    double A ;

    g0 = (X-p2.x())*(p2.y()-p3.y()) - (Y-p2.y())*(p2.x()-p3.x()) ;
    g1 = (X-p1.x())*(p3.y()-p1.y()) - (Y-p1.y())*(p3.x()-p1.x()) ;
 
    A = 2*area() ;

    g0 /= A ; g1 /= A ; g2 = 1.0 - g0 - g1 ;
}

}
