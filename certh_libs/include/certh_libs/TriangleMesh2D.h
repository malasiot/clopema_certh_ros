#ifndef _TRIANGLE_MESH_2D_H
#define _TRIANGLE_MESH_2D_H

#include <certh_libs/Point2D.h>
#include <certh_libs/Polygon2D.h>
#include <certh_libs/TriangleMeshTopology.h>

#include <vector>

namespace certh_libs {

// Finite element triangular mesh.

    class TriangleMesh2D
	{
	public:

        TriangleMesh2D() {}
        ~TriangleMesh2D() {}

        void addFace(int v1, int v2, int v3) ;
        void getTopology( TriangleMeshTopology &topo) const ;

        struct Face {
            int v1, v2, v3 ;
        };

        std::vector<Point2D> coords ;
        std::vector<Face> faces ;
	} ;

    // Delaunay triangulation of a list of points

    void triangulate(const std::vector<Point2D> &pts, TriangleMesh2D &mesh_out) ;

    // Perform constrained triangulation where additional points may be inserted
    // to keep the triangle area bellow a threshold

    void triangulate(const std::vector<Point2D> &pts, double area,
                                      TriangleMesh2D &mesh_out);


    // Same as above but also interpolates attributes over inserted points.
    // The parameter in_attrs may be used to specify a nPts * nAttr list of
    // attributes, i.e. each input point is associated with nAttr scalar values.
    // The interpolated attributes are returned on out_attrs

    void triangulate(const std::vector<Point2D> &pts, double area,
                            const std::vector<double> &in_attrs,
                            TriangleMesh2D &mesh_out,
                            std::vector<double> &out_attrs);

    // Delauany triangulation of a polygon possible non-convex
    void triangulate(const Polygon2D &poly, TriangleMesh2D &mesh_out) ;

    // Constrained polygon triangulation
    void triangulate(const Polygon2D &poly, double area, TriangleMesh2D &mesh_out) ;

    // Constrained polygon triangulation with attribute interpolation
    void triangulate(const Polygon2D &poly, double area,
        const std::vector<double> in_attrs,  TriangleMesh2D &mesh_out,
        std::vector<double> &attrs_out) ;

    // constraint polygon triangulation with boundary polygon and a list of segments
    void triangulate(const Polygon2D &bpoly, const std::vector< std::vector<Point2D> > &plist,  double area,  TriangleMesh2D &mesh_out) ;

    // Compute the convex hull of a point list
    void convexHull(const std::vector<Point2D> &pts, Polygon2D &poly) ;

} // namespace cpm



#endif
