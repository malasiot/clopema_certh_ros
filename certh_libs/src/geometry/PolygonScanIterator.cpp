/*
 * Concave Polygon Scan Conversion
 * by Paul Heckbert
 * from "Graphics Gems", Academic Press, 1990
 */


#include <certh_libs/PolygonScanner.h>

#include <vector>
#include <string.h>


using namespace std ;

namespace certh_libs {

//const Polygon2D *pt ;
/*
int PolygonScanIterator::compareInd(const void *u, const void *v)
{
    return (*pt)[*(int *)u].y < (*pt)[*(int *)v].y ? -1 : 1;
}
*/


int PolygonScanIterator::compareActive(const void *u, const void *v)
{
    return ((_edge *)u)->x < ((_edge *)v)->x ? -1 : 1;
}


struct CompareInd {

    CompareInd(const Polygon2D &poly): pt(poly) {}

    bool operator () ( int u, int v) {
        return pt[u].y() < pt[v].y() ;
    }

    const Polygon2D &pt ;

};

PolygonScanIterator::PolygonScanIterator(const Polygon2D &pt, double *at, int ne): poly(pt), attr(at), nelem(ne)
{
    n = poly.getNumPoints() ;
   
    assert( n > 2 ) ;

    ind = new int [n] ;
    active = new _edge [n] ;

    reset() ;
}

PolygonScanIterator::~PolygonScanIterator()
{
    delete [] ind ;
    delete [] active ;
}

void PolygonScanIterator::incrementX()
{
    double frac = 1.0 ;

    double dx = active[jscan+1].x - active[jscan].x ;
    if (dx == 0.) dx = 1. ;
    frac = x+.5 - active[jscan].x ;

    double *pda = da, *pa = a, *p2 = active[jscan+1].a, *p1 = active[jscan].a ;
  
    for( int j=0 ; j<nelem ; j++, pa++, pda++, p1++, p2++ ) {
        *pda = ( *p2 - *p1)/dx ;
        *pa = *p1 + *pda * frac;
    }
}
  
void PolygonScanIterator::reset()
{

    for (int k=0; k<n; k++) ind[k] = k;

    std::sort(ind, ind + n, CompareInd(poly)) ;

    //pt = &poly ;
    //qsort(ind, n, sizeof ind[0], compareInd) ;
  
    gk = 0 ; nact = 0;
 
    y0 = (int)ceil(poly[ind[0]].y()-.5);
    y1 = (int)floor(poly[ind[n-1]].y()-.5);

    y = y0 ; jscan = 0 ;

    valid = true ;

    findActiveScans() ;

    findNextValidScan() ;

    if ( y>y1 || xl>xr )
    {
        valid = false ;
        return ;
    }

    x = xl ;

    incrementX() ;
}

void PolygonScanIterator::findNextValidScan()
{

    while ( y <= y1 )
    {
        jscan = 0 ;

        while ( jscan < nact )
        {
            xl = (int)ceil(active[jscan].x-.5);
            xr = (int)floor(active[jscan+1].x-.5);

            if ( xl > xr ) {
                active[jscan].x += active[jscan].dx;
                active[jscan+1].x += active[jscan+1].dx;
                jscan += 2 ;
            }
            else return ;

        } ;

        ++y ;

        if ( y<= y1 )
        {
            findActiveScans() ;
        }
    }

}


void PolygonScanIterator::findActiveScans()
{
    int i, j ;

    for ( ; gk<n && poly[ind[gk]].y()<=y+.5 ; gk++ )
    {
        i = ind[gk];
        j = i>0 ? i-1 : n-1;

        if (poly[j].y() <= y-.5) cdelete(j);
        else if (poly[j].y() > y+.5) cinsert(j, y);

        j = i<n-1 ? i+1 : 0;

        if (poly[j].y() <= y-.5) cdelete(i) ;
        else if (poly[j].y() > y+.5) cinsert(i, y);
    }

 //   pt = &poly ;
    qsort(active, nact, sizeof active[0], compareActive);
}

PolygonScanIterator::operator int () const { return valid ; }
 
PolygonScanIterator & PolygonScanIterator::operator++() 
{
    valid = false ;

    ++x ;

    double *pa = a, *pda = da ;
    int j ;
    for(  j=0 ; j<nelem ; j++, pa++, pda++ ) *pa += *pda ;

    if ( x <= xr ) {
        valid = true ;
        return *this ;
    }


    active[jscan].x += active[jscan].dx;
    active[jscan+1].x += active[jscan+1].dx;
  
    pa = active[jscan].a ; pda = active[jscan].da  ;
    for( j=0 ; j<nelem ; j++, pa++, pda++ ) *pa += *pda ;
    pa = active[jscan+1].a ; pda = active[jscan+1].da  ;
    for( j=0 ; j<nelem ; j++, pa++, pda++ ) *pa += *pda ;

    jscan += 2 ;

    if ( jscan < nact )
    {
        x = xl = (int)ceil(active[jscan].x-.5);
        xr = (int)floor(active[jscan+1].x-.5);

        incrementX() ;
    
        valid = true ;
        return *this ;
    }
    else
    {
        ++y ;
        findActiveScans() ;
        findNextValidScan() ;

        if ( y <= y1 ) {
            x = xl ;
            incrementX() ;
            valid = true ;
            return *this ;
        }
        else valid = false ;

        return *this ;
    }

    return *this ;
}

void PolygonScanIterator::cdelete(int i)
{
    int j;

    for (j=0; j<nact && active[j].i!=i; j++) ;
    if (j>=nact) return ;
    nact-- ;
    memcpy(&active[j], &active[j+1], (nact-j)*sizeof active[0]);
}

void PolygonScanIterator::cinsert(int i, int y)
{
    int j;
    double dx;
    Point2D p, q ;
    double ap[_MAX_ATTR], aq[_MAX_ATTR], day ;

    j = i<poly.getNumPoints()-1 ? i+1 : 0 ;
  
    int ioff = i * nelem, joff = j * nelem ;

    if (poly[i].y() < poly[j].y()) {
        p = poly[i]; q = poly[j];
        memcpy(ap, attr + ioff, sizeof(double) * nelem ) ;
        memcpy(aq, attr + joff, sizeof(double) * nelem ) ;
    }
    else {
        p = poly[j]; q = poly[i];
        memcpy(ap, attr + joff, sizeof(double) * nelem ) ;
        memcpy(aq, attr + ioff, sizeof(double) * nelem ) ;
    }
 

    active[nact].dx = dx = (q.x()-p.x())/(q.y()-p.y());
    active[nact].x = dx*(y+.5-p.y())+p.x();
    active[nact].i = i;

    double *paq = aq, *pap = ap, *pa = active[nact].a, *pda = active[nact].da ;

    for( j=0 ; j<nelem ; j++, pap++, paq++, pa++, pda++ )
    {
        *pda = day = (*paq - *pap)/( q.y() - p.y() ) ;
        *pa = day * (y + .5 -p.y()) + *pap ;
    }

    nact++;
}

void getPointsInPoly(const Polygon2D &poly, std::vector<Point> &pts)
{
    PolygonScanIterator it(poly) ;

    while (it) {
        pts.push_back(*it) ;
        ++it ;
    }

}

} // namespace cpm
