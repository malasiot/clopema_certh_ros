#include <certh_libs/TriangleMesh2D.h>


#include <fstream>
#include <map>

#include <string.h>

using namespace std ;

extern "C" {
#define REAL double
#define VOID void
#define ANSI_DECLARATORS
#include "triangle.h"
#undef triangulate
}

#undef min
#undef max

#pragma warning(disable: 4244 4018)

namespace certh_libs {

void TriangleMesh2D::addFace(int v1, int v2, int v3)
{
    Face f ;
    f.v1 = v1 ; f.v2 = v2 ; f.v3 = v3 ;

    faces.push_back(f) ;

}


void TriangleMesh2D::getTopology( TriangleMeshTopology &topo ) const
{
    for(int i=0 ; i<faces.size() ; i++ )
    {
        const Face &f = faces[i] ;
        topo.addFace(f.v1, f.v2, f.v3) ;
    }
}

////////////////////////////////////////////////////////////////////////////


void triangulatePts(const std::vector<Point2D> &pts, double area,
                   const std::vector<double> &attrs, std::vector<double> &out_attrs,
                    TriangleMesh2D &mesh_out)
{
    char triswitches[20] ;
    int i, k ;

    if ( area > 0.0 ) sprintf(triswitches, "zqQa%f", area) ;
    sprintf(triswitches, "zQ") ;

    struct triangulateio trsin, trsout ;
    int n ;

    n = trsin.numberofpoints = pts.size() ;
    trsin.pointlist = (REAL *) malloc(n * 2 * sizeof(REAL));

    int _nattr ;
    double *_attrList ;

    if ( attrs.empty() )
    {
        _nattr = 0 ;
        _attrList = 0 ;
    }
    else
    {
        _nattr = attrs.size()/pts.size() ;
        _attrList = new double [attrs.size()] ;

        std::copy(attrs.begin(), attrs.end(), _attrList);

    }

    trsin.numberofpointattributes = _nattr ;
    trsin.pointattributelist = _attrList ;

    for(i=0, k=0 ; i<n ; i++, k+=2)
    {
        const Point2D &p = pts[i] ;
        trsin.pointlist[k]   = p.x() ;
        trsin.pointlist[k+1] = p.y() ;
    }

    trsin.pointmarkerlist = NULL ;

    trsin.numberofsegments = 0 ;
    trsin.numberofholes = 0 ;
    trsin.numberofregions = 0 ;

    trsout.pointlist = (REAL *)NULL ;
    trsout.pointattributelist = (REAL *)NULL ;
    trsout.pointmarkerlist = (int *)NULL ;
    trsout.trianglelist = (int *) NULL;
    trsout.triangleattributelist = (REAL *) NULL;

    _triangulate(triswitches, &trsin, &trsout, NULL) ;

    for(i=0, k=0 ; i<trsout.numberofpoints ; i++, k+=2)
    {
        double x, y ;

        x = trsout.pointlist[k] ;
        y = trsout.pointlist[k+1] ;

        mesh_out.coords.push_back(Point2D(x, y)) ;
    }

    for(i=0, k=0 ; i<trsout.numberoftriangles ; i++, k+=3)
    {
        TriangleMesh2D::Face f ;

        f.v1 = trsout.trianglelist[k] ;
        f.v2 = trsout.trianglelist[k+1] ;
        f.v3 = trsout.trianglelist[k+2] ;

        mesh_out.faces.push_back(f) ;
    }

    if ( _nattr != 0 && _attrList != NULL )
    {
        int as = _nattr * trsout.numberofpoints ;
        out_attrs.resize(as) ;
        std::copy(trsout.pointattributelist, trsout.pointattributelist+as,
                  out_attrs.begin()) ;

    }

    free(trsin.pointlist) ;
    free(trsout.pointlist) ;

    free(trsout.pointmarkerlist) ;
    free(trsout.trianglelist) ;

    if ( trsout.pointattributelist ) free(trsout.pointattributelist) ;
    if ( _attrList ) delete _attrList ;

}


void triangulate(const std::vector<Point2D> &pts, TriangleMesh2D &mesh_out)
{
   std::vector<double> attr_in, attr_out ;

   triangulatePts(pts, 0.0, attr_in, attr_out, mesh_out) ;
}

void triangulate(const std::vector<Point2D> &pts, double area,
                                  TriangleMesh2D &mesh_out)
{
    std::vector<double> attr_in, attr_out ;

    triangulatePts(pts, area, attr_in, attr_out, mesh_out) ;
}

void triangulate(const std::vector<Point2D> &pts, double area,
                        const std::vector<double> &in_attrs,
                        TriangleMesh2D &mesh_out,
                        std::vector<double> &out_attrs)
{
    triangulatePts(pts, area, in_attrs, out_attrs, mesh_out) ;
}

///////////////////////////////////////////////////////////////////////////

void triangulatePoly(const Polygon2D &bpoly,
                     const std::vector<Point2D> &plist,
                     double area, const std::vector<double> &attrs,
                     std::vector<double> &out_attrs,
                     TriangleMesh2D &mesh_out)
{
    char triswitches[20] ;
    int i, k ;

    if ( area > 0.0 ) sprintf(triswitches, "zqpQa%f", area) ;
    else sprintf(triswitches, "zpQ") ;

    struct triangulateio trsin, trsout ;
    int n, np ;

    np = bpoly.getNumPoints() ;
    n = plist.size() ;
    trsin.numberofpoints = n + np ;

    int _nattr ;
    double *_attrList ;

    if ( attrs.empty() )
    {
        _nattr = 0 ;
        _attrList = 0 ;
    }
    else
    {
        _nattr = attrs.size()/plist.size() ;
        _attrList = new double [attrs.size()] ;

        std::copy(attrs.begin(), attrs.end(), _attrList);

    }

    trsin.numberofpointattributes = _nattr ;
    trsin.pointattributelist = _attrList ;
    trsin.pointlist = (REAL *) malloc((n + np) * 2 * sizeof(REAL));
    trsin.segmentmarkerlist = (int *)NULL ;

    for(i=0, k=0 ; i<np ; i++, k+=2)
    {
        const Point2D &p = bpoly[i] ;

        trsin.pointlist[k]   = p.x() ;
        trsin.pointlist[k+1] = p.y() ;
    }

    for(i=0 ; i<n ; i++, k+=2)
    {
        const Point2D &p = plist[i] ;
        trsin.pointlist[k]   = p.x() ;
        trsin.pointlist[k+1] = p.y() ;
    }

    trsin.pointmarkerlist = NULL ;
    trsin.numberofsegments = np ;
    trsin.segmentlist = (int *)malloc(2 * trsin.numberofsegments * sizeof(int)) ;

    for(i=0, k=0 ; i<trsin.numberofsegments-1 ; i++, k+=2)
    {
        trsin.segmentlist[k]  = i ;
        trsin.segmentlist[k+1]  = i + 1 ;
    }

    trsin.segmentlist[k] = np - 1 ;
    trsin.segmentlist[k+1] = 0 ;

    trsin.numberofholes = 0 ;
    trsin.numberofregions = 0 ;

    trsout.pointlist = (REAL *)NULL ;
    trsout.pointattributelist = (REAL *)NULL ;
    trsout.pointmarkerlist = (int *)NULL ;
    trsout.trianglelist = (int *) NULL;
    trsout.triangleattributelist = (REAL *) NULL;
    trsout.segmentlist = (int *)NULL ;
    trsout.segmentmarkerlist = (int *)NULL ;

    _triangulate(triswitches, &trsin, &trsout, NULL) ;

    for(i=0, k=0 ; i<trsout.numberofpoints ; i++, k+=2)
    {
        double x = trsout.pointlist[k] ;
        double y = trsout.pointlist[k+1] ;

        mesh_out.coords.push_back(Point2D(x, y)) ;
    }

    for(i=0, k=0 ; i<trsout.numberoftriangles ; i++, k+=3)
    {
        TriangleMesh2D::Face f ;

        f.v1 = trsout.trianglelist[k] ;
        f.v2 = trsout.trianglelist[k+1] ;
        f.v3 = trsout.trianglelist[k+2] ;

        mesh_out.faces.push_back(f) ;
    }

    if ( _nattr != 0 && _attrList != NULL )
    {
        int as = _nattr * trsout.numberofpoints ;
        out_attrs.resize(as) ;
        std::copy(trsout.pointattributelist, trsout.pointattributelist+as,
                  out_attrs.begin()) ;

    }

    free(trsin.pointlist) ;
    free(trsin.segmentlist) ;
    free(trsout.pointlist) ;
    free(trsout.trianglelist) ;
    free(trsout.pointmarkerlist) ;

    if ( trsout.pointattributelist ) free(trsout.pointattributelist) ;
    if ( _attrList ) delete _attrList ;

}

void triangulate(const Polygon2D &poly, TriangleMesh2D &mesh_out)
{
    std::vector<double> attrs_in, attrs_out ;
    std::vector<Point2D> pts ;

    triangulatePoly(poly, pts, 0.0, attrs_in, attrs_out, mesh_out) ;
}

void triangulate(const Polygon2D &poly, double area, TriangleMesh2D &mesh_out)
{
    std::vector<double> attrs_in, attrs_out ;
    std::vector<Point2D> pts ;

    triangulatePoly(poly, pts, area, attrs_in, attrs_out, mesh_out) ;

}

void triangulate(const Polygon2D &poly, double area,
    const std::vector<double> attrs_in,  TriangleMesh2D &mesh_out,
    std::vector<double> &attrs_out)
{
    std::vector<Point2D> pts ;

    triangulatePoly(poly, pts, area, attrs_in, attrs_out, mesh_out) ;

}

///////////////////////////////////////////////////////////////////////////

void triangulate(const Polygon2D &bpoly, const vector< vector<Point2D> > &plist,  double area,  TriangleMesh2D &mesh_out)
{
    char triswitches[20] ;
    int i, k ;

    if ( area > 0.0 ) sprintf(triswitches, "zpqQVa%f", area) ;
    else sprintf(triswitches, "zpqQV") ;

    struct triangulateio trsin, trsout ;
    int n = 0, nseg = 0, np ;

    np = bpoly.getNumPoints() ;

    for(int i=0 ; i<plist.size() ; i++ ) {
        n += plist[i].size() ;
        nseg += plist[i].size() - 1 ;
    }

    trsin.numberofpoints = n + np ;

    trsin.numberofpointattributes = 0 ;
    trsin.pointattributelist = NULL ;
    trsin.pointlist = (REAL *) malloc((n + np) * 2 * sizeof(REAL));
    trsin.segmentmarkerlist = (int *)NULL ;

    for(i=0, k=0 ; i<np ; i++, k+=2)
    {
        const Point2D &p = bpoly[i] ;

        trsin.pointlist[k]   = p.x() ;
        trsin.pointlist[k+1] = p.y() ;
    }

    for(int j=0 ; j<plist.size() ; j++)
    {
        const vector<Point2D> &seg = plist[j] ;

        for(i=0 ; i<seg.size() ; i++, k+=2)
        {
            const Point2D &p = seg[i] ;
            trsin.pointlist[k]   = p.x() ;
            trsin.pointlist[k+1] = p.y() ;
        }
    }

    trsin.pointmarkerlist = NULL ;
    trsin.numberofsegments = np + nseg;
    trsin.segmentlist = (int *)malloc(2 * trsin.numberofsegments * sizeof(int)) ;

    for(i=0, k=0 ; i<np-1 ; i++, k+=2)
    {
        trsin.segmentlist[k]  = i ;
        trsin.segmentlist[k+1]  = i + 1 ;
    }

    trsin.segmentlist[k] = np - 1 ;
    trsin.segmentlist[k+1] = 0 ;

    ++k ; ++k ;

    int r = 0 ;

    for(int j=0 ; j<plist.size() ; j++)
    {
        const vector<Point2D> &seg = plist[j] ;

        for(i=0 ; i<seg.size()-1 ; i++, k+=2, r++)
        {
            trsin.segmentlist[k]  = r+np ;
            trsin.segmentlist[k+1]  = r+np + 1 ;

        }
        ++r ;
    }

    trsin.numberofholes = 0 ;
    trsin.numberofregions = 0 ;

    trsout.pointlist = (REAL *)NULL ;
    trsout.pointattributelist = (REAL *)NULL ;
    trsout.pointmarkerlist = (int *)NULL ;
    trsout.trianglelist = (int *) NULL;
    trsout.triangleattributelist = (REAL *) NULL;
    trsout.segmentlist = (int *)NULL ;
    trsout.segmentmarkerlist = (int *)NULL ;

    _triangulate(triswitches, &trsin, &trsout, NULL) ;

    for(i=0, k=0 ; i<trsout.numberofpoints ; i++, k+=2)
    {
        double x = trsout.pointlist[k] ;
        double y = trsout.pointlist[k+1] ;

        mesh_out.coords.push_back(Point2D(x, y)) ;
    }

    for(i=0, k=0 ; i<trsout.numberoftriangles ; i++, k+=3)
    {
        TriangleMesh2D::Face f ;

        f.v1 = trsout.trianglelist[k] ;
        f.v2 = trsout.trianglelist[k+1] ;
        f.v3 = trsout.trianglelist[k+2] ;

        mesh_out.faces.push_back(f) ;
    }


    free(trsin.pointlist) ;
    free(trsin.segmentlist) ;
    free(trsout.pointlist) ;
    free(trsout.trianglelist) ;
    free(trsout.pointmarkerlist) ;


}



/////////////////////////////////////////////////////////////////////////////

void convexHull(const std::vector<Point2D> &plist, Polygon2D &poly)
{
    char triswitches[20] ;
    int i, k ;

    sprintf(triswitches, "zcqQY") ;

    struct triangulateio trsin, trsout ;
    int n ;

    n = trsin.numberofpoints = plist.size() ;
    trsin.pointlist = (REAL *) malloc(n * 2 * sizeof(REAL));

    trsin.numberofpointattributes = 0 ;
    trsin.pointattributelist = NULL ;

    for(i=0, k=0 ; i<n ; i++, k+=2)
    {
        const Point2D &p = plist[i] ;
        trsin.pointlist[k]   = p.x() ;
        trsin.pointlist[k+1] = p.y() ;
    }

    trsin.pointmarkerlist = NULL ;

    trsin.numberofsegments = 0 ;
    trsin.numberofholes = 0 ;
    trsin.numberofregions = 0 ;

    trsout.pointlist = (REAL *)NULL ;
    trsout.pointattributelist = (REAL *)NULL ;
    trsout.pointmarkerlist = (int *)NULL ;
    trsout.trianglelist = (int *) NULL;
    trsout.triangleattributelist = (REAL *) NULL;
    trsout.segmentlist = (int *) NULL;
    trsout.segmentmarkerlist = (int *) NULL;

    _triangulate(triswitches, &trsin, &trsout, NULL) ;

    for(i=0, k=0 ; i<trsout.numberofsegments ; i++, k+=2)
    {
        int i1, i2 ;

        i1 = trsout.segmentlist[k] ;
        i2 = trsout.segmentlist[k+1] ;

        if ( trsout.segmentmarkerlist[i] == 1 ) {
            poly.addPoint(Point2D(trsout.pointlist[2*i1], trsout.pointlist[2*i1+1]) ) ;
        }
    }

    free(trsin.pointlist) ;
    free(trsout.pointlist) ;
    free(trsout.pointmarkerlist) ;
    free(trsout.trianglelist) ;
    free(trsout.segmentlist) ;
    free(trsout.segmentmarkerlist) ;
}


}
