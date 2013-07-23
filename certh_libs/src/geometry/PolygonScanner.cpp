/*
 * Concave Polygon Scan Conversion
 * by Paul Heckbert
 * from "Graphics Gems", Academic Press, 1990
 */

#include <certh_libs/PolygonScanner.h>
#include <certh_libs/Point.h>

#include <vector>
#include <string.h>

using namespace std ;

namespace vl {

#define ALLOC(ptr, type, n)  assert(ptr = new type [n])

static const Polygon2D *pt ;

int PolygonScanner::compareInd(const void *u, const void *v)
{
    return (*pt)[*(int *)u].y < (*pt)[*(int *)v].y ? -1 : 1;
}

int PolygonScanner::compareActive(const void *u, const void *v)
{
    return ((_edge *)u)->x < ((_edge *)v)->x ? -1 : 1;
}

void PolygonScanner::init()
{
    pt = &poly ;

    n = poly.getNumPoints() ;

    if (n<=2) return;

    ind = new int [n] ;
    active = new _edge [n] ;

    for(k=0; k<n; k++) ind[k] = k;

    qsort(ind, n, sizeof ind[0], compareInd) ;

    nact = 0; k=0 ;

    y0 = (int)ceil(poly[ind[0]].y-.5);
    y1 = (int)floor(poly[ind[n-1]].y-.5);
}

void PolygonScanner::findActiveScans(int y)
{
    int i, j ;
    for (; k<n && poly[ind[k]].y<=y+.5; k++)
    {

        i = ind[k];
        j = i>0 ? i-1 : n-1;

        if (poly[j].y <= y-.5) cdelete(j);
        else if (poly[j].y > y+.5) cinsert(j, y);

        j = i<n-1 ? i+1 : 0;

        if (poly[j].y <= y-.5) cdelete(i) ;
        else if (poly[j].y > y+.5) cinsert(i, y);
    }

    qsort(active, nact, sizeof active[0], compareActive);


}

void PolygonScanner::scanOneLine(int y)
{
    int  i, j, xl, xr ;

    findActiveScans(y) ;

    for (j=0; j<nact; j+=2)
    {
        xl = (int)ceil(active[j].x-.5);
        xr = (int)floor(active[j+1].x-.5);

        if ( xl<=xr ) this->scanLine(y, xl, xr) ;

        active[j].x += active[j].dx;
        active[j+1].x += active[j+1].dx;
    }
}

void PolygonScanner::scan()
{
    int k, y, i, j, xl, xr;

    init() ;

    for (y=y0; y<=y1; y++)
       scanOneLine(y) ;

}

void PolygonScanner::cdelete(int i)
{
    int j;

    for (j=0; j<nact && active[j].i!=i; j++) ;
    if (j>=nact) return ;
    nact-- ;
    memcpy(&active[j], &active[j+1], (nact-j)*sizeof active[0]);
}

void PolygonScanner::cinsert(int i, int y)
{
    int j;
    double dx;
    Point2D p, q ;

    j = i<poly.getNumPoints()-1 ? i+1 : 0 ;
  
    if (poly[i].y < poly[j].y) {p = poly[i]; q = poly[j];}
    else {p = poly[j]; q = poly[i];}

    active[nact].dx = dx = (q.x-p.x)/(q.y-p.y);
    active[nact].x = dx*(y+.5-p.y)+p.x;
    active[nact].i = i;
  
    nact++;
}

class PolyScannerSimple: public PolygonScanner
{
    public:

    PolyScannerSimple(const Polygon2D &poly, vector<Point> &pts_):
        PolygonScanner(poly), pts(pts_) {}

    void scanLine(int y, int xl, int xr) {
        for( int j=xl ; j<=xr ; j++ ) pts.push_back(Point(j, y)) ;
  }

  private:

  vector<Point> &pts ;
} ;

void getPointsInPoly(const Polygon2D &poly, vector<Point> &pts)
{
    PolyScannerSimple sc(poly, pts) ;
    sc.scan() ;
}

} // namespace vl
