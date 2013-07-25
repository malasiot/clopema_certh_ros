#ifndef __POLYGON_SCANNER_H
#define __POLYGON_SCANNER_H

#include <certh_libs/Polygon2D.h>
#include <certh_libs/Point.h>

#include <vector>

const int _MAX_ATTR = 20 ;

namespace certh_libs {

class PolygonScanIterator
{
  public:

    // Iterates over all pixels inside a concave polygon.
    // One may also provide attributes for polygon vertices. Each attribute is
    // nelem-dimensional vector. Vertex attributes are stored sequentially in
    // array att. Then the scanner linearly interpolates attributes at pixel centers

    PolygonScanIterator(const Polygon2D &p, double *att = NULL, int nelem = 0) ;
    ~PolygonScanIterator() ;

    void reset() ;

    // Checks whether the iterator is still valid
    operator int () const ;

    // Go to the next pixel
    PolygonScanIterator & operator++() ;
  
    // Get the coordinates of the current pixel
    Point operator *() const { return Point(x, y) ; }

  // Get the coordinate i of the interpolate attribute vector at the current pixel 
    const double operator() (int i) const { return a[i] ; }

    private:
  
    struct _edge {
        double x, a[_MAX_ATTR] ;
        double dx, da[_MAX_ATTR];
        int i;
    } *active ;

    Polygon2D poly ;
    int nact, jscan ;
    int gk, n, y0, y1, x, y, xl, xr, nelem ;
    double a[_MAX_ATTR], da[_MAX_ATTR];
    int *ind ;
    bool valid ;
    double *attr ;

    static int compareInd(const void *, const void *) ;
    static int compareActive(const void *, const void *) ;

    void cdelete(int) ;
    void cinsert(int, int) ;

    void findActiveScans() ;
    void findNextValidScan() ;
    void incrementX() ;

} ;


class TriangleScanIterator: public PolygonScanIterator
{
public:
 
    TriangleScanIterator(const Triangle2D &tr, double *at = NULL, int nelem = 0):
        PolygonScanIterator(Polygon2D(tr), at, nelem) {}
 
} ;


void getPointsInPoly(const Polygon2D &poly, std::vector<Point> &pts) ;
}
    
#endif
