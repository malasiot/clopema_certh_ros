#include <certh_libs/Polygon2D.h>
#include <certh_libs/Triangle2D.h>
#include <certh_libs/TriangleMesh2D.h>

using namespace std ;

#undef min
#undef max

namespace certh_libs {

Polygon2D::Polygon2D(const Point2D *pl, int n): plist(), trg(NULL)
{
    for(int i=0 ; i<n ; i++) plist.push_back(pl[i]) ;
}

Polygon2D::~Polygon2D() {
    if ( trg ) delete trg ;
}

double Polygon2D::area() const
{
    double s ;
    int i, i1 ;
    int n = getNumPoints() ;

    s = 0 ;

    for( i=0 ; i<n ; i++ )
    {
        i1 = (i+1)%n ;
        const Point2D &pi = plist[i], &pi1 = plist[i1] ;
        s += pi.x()*pi1.y() - pi1.x()*pi.y() ;
    }

    return 0.5*fabs(s) ;
}

Rectangle2D Polygon2D::boundingBox() const
{
    int i ;
    Point2D pmin = plist[0], pmax = plist[0] ;

    for( i=1 ; i<getNumPoints() ; i++ )
    {
        pmin = min(pmin,plist[i]) ;
        pmax = max(pmax,plist[i]) ;
    }

    return Rectangle2D(pmin, pmax) ;
}

Polygon2D::ICode Polygon2D::whereIs(const Point2D &P)
{
    int i ;
    int inpoly ;
    int n = getNumPoints() ;

    if ( !trg )
    {
        trg = new TriangleMesh2D ;
        triangulate(*this, *trg) ;
    }

    inpoly = PtOutPolygon ;

    for( i=0 ; i<trg->faces.size() ; i++ )
    {
        const TriangleMesh2D::Face &face = trg->faces[i] ;

        Triangle2D tr(plist[face.v1],plist[face.v2],
                  plist[face.v3]) ;

        Triangle2D::ICode intri = tr.whereIs(P) ;

        if ( intri == Triangle2D::PtInTriangle ) return PtInPolygon ;
        if ( intri == Triangle2D::PtOnVertex ) return PtOnVertex ;
        else if ( intri == Triangle2D::PtOnEdge )
        {
            if ( inpoly == PtOnEdge ) return PtInPolygon ;
            else return PtOnEdge ;
        }
    }

    return PtOutPolygon ;
}

bool Polygon2D::contains(const Point2D &p)
{
  return ( whereIs(p) == PtOutPolygon ) ? false : true ;
}



}
