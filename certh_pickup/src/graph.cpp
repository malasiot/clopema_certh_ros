#include <cv.h>
#include <highgui.h>

#include <certh_libs/TriangleMesh2D.h>
#include <certh_libs/PolygonScanner.h>
#include <certh_libs/cvHelpers.h>

#include <fstream>
#include <functional>
#include <boost/unordered_map.hpp>

#include "sg/segment-graph.h"
#include "sg/disjoint-set.h"

using namespace certh_libs;
using namespace std ;


double computeEdgeCost(const cv::Mat &gim_, const cv::Mat &dim_,
                       const Point2D &p1, const Point2D &p2)
{
    cv::Mat_<float> gim(gim_) ;
    cv::Mat_<ushort> dim(dim_) ;

    vector<cv::Point> pixels ;
    getScanLine(cv::Point(p1.x(), p1.y()), cv::Point(p2.x(), p2.y()), pixels) ;

    double gcost = 0.0 ;

    for(int i=0 ; i<pixels.size() ; i++)
    {
        const cv::Point &p0 = pixels[i] ;

        // for the gradient image just  accumulate magnitudes

        gcost += gim[p0.y][p0.x] ;
    }

    gcost /= pixels.size() ;

    double dcost = 0.0 ;

    for(int i=1 ; i<pixels.size()-1 ; i++)
    {
        const cv::Point &p0 = pixels[i-1] ;
        const cv::Point &p1 = pixels[i] ;
        const cv::Point &p2 = pixels[i+1] ;

        ushort d0 = dim[p0.y][p0.x] ;
        ushort d1 = dim[p1.y][p1.x] ;
        ushort d2 = dim[p2.y][p2.x] ;


        // for the depth image use second order direvative (we ignore zero values for the moment)

        if ( d0 == 0 || d1 == 0 || d2 == 0 ) dcost += 100 ;
        else dcost += fabs(d2 - d1 - d1 + d0) ;
    }

    dcost /= pixels.size()-2 ;



  //  const double gWeight = 0.3 ;

      const double gWeight = 0 ;
    const double dWeight = 0.05;

    return gcost * gWeight + dcost * dWeight ;



}


static void scanTriangle(cv::Mat_<ushort> &im, cv::Mat_<ushort> &dim, unsigned short label, const Point2D &p1, const Point2D &p2, const Point2D &p3)
{
    TriangleScanIterator it(Triangle2D(p1, p2, p3)) ;

    while ( it )
    {
        Point p = *it ;

        int x = p.x(), y = p.y() ;

        if ( dim[y][x] != 0 )
            im[y][x] = label ;
        ++it ;

    }

}

struct cEdge {
    int f1, f2 ;
    int v ;
};

struct vEdge {
    int v, vp, vn ;
};


struct BoundaryInfo {

    std::vector<cEdge> edges ;

};

typedef std::multimap<std::pair<int, int>, cEdge > EdgeMap ;

struct eFace {
    eFace(): e1(-1), e2(-1) {}
    int e1, e2 ;
};

struct fEdge {
    fEdge(): f1(-1), f2(-1) {}
    int f1, f2 ;
};

#define DUMMY_FACE 100000

static void makeChain(const BoundaryInfo &binfo, const std::map<int, int> &junctions, std::vector<int> &chain )
{
    int bface_count = DUMMY_FACE ;

    typedef std::map<int, eFace> FaceEdgeMap ;
    FaceEdgeMap femap ; // collect faces and their adjacent edges for fast indexing

    typedef std::map<int, fEdge> EdgeFaceMap ;
    EdgeFaceMap efmap ;

    for( int i=0 ; i<binfo.edges.size(); i++ )
    {
        const cEdge &edge = binfo.edges[i] ;

        int f1 = edge.f1, f2 = edge.f2 ;

        if ( f1 == -1 ) f1 = bface_count++ ;
        if ( f2 == -1 ) f2 = bface_count++ ;

        eFace &ef1 = femap[f1] ;

        if ( ef1.e1 == -1 ) ef1.e1 = i ;
        else ef1.e2 = i ;

        eFace &ef2 = femap[f2] ;

        if ( ef2.e1 == -1 ) ef2.e1 = i ;
        else ef2.e2 = i ;


        fEdge &e = efmap[i] ;
        e.f1 = f1 ;
        e.f2 = f2 ;
    }

    // find start and end faces

    std::set<int> start_faces ;

    //int start_face = -1, end_face = -1 ;
    int start_junction = -1, end_junction = -1;

    FaceEdgeMap::const_iterator it = femap.begin() ;

    for( ; it != femap.end() ; ++it )
    {
        int face_idx = (*it).first ;
        const eFace &face = (*it).second ;
        if ( face.e1 == -1 ) {
            start_faces.insert(face_idx) ;
        }
        else if ( face.e2 == -1 ) {
            start_faces.insert(face_idx) ;
        }
    }



    int nChains = start_faces.size()/2 ;
    bool isCycle = false ;

    // we have to sort the faces into a chain

    if ( nChains > 1 )
    {
        cout << "ok here" << endl ;
    }
    else if ( nChains == 0 )
    {
        isCycle = true ;
        nChains = 1;
        cout << "ok here2" << endl ;
    }

    for(int i=0 ; i<nChains ; i++)
    {
        int start_face ;

        if ( !isCycle )
        {
            std::set<int>::const_iterator sfit = start_faces.begin() ;

            start_face = (*sfit) ;
            start_faces.erase(sfit) ;
        }
        else
        {
            start_face = (*femap.begin()).first ;
        }


        int cf = start_face ;
        int pf = -1 ;

        std::vector<int> fchain ;

        while (1)
        {
            eFace &face = femap[cf] ;

            fchain.push_back(cf) ;

            if ( isCycle && cf == start_face && pf != -1 )
            {
                break ;

            }
            else if ( start_faces.count(cf) ) {
                start_faces.erase(cf) ;
                break ;
            }

            if ( face.e1 != -1 )
            {
                fEdge &fedge = efmap[face.e1] ;

                if ( fedge.f1 != cf && fedge.f1 != pf )
                {
                    int nf = fedge.f1 ;
                    pf = cf ;
                    cf = nf ;
                    continue ;
                }
                else  if ( fedge.f2 != cf && fedge.f2 != pf )
                {
                    int nf = fedge.f2 ;
                    pf = cf ;
                    cf = nf ;
                    continue ;
                }

            }

            if ( face.e2 != -1 )
            {
                fEdge &fedge = efmap[face.e2] ;

                if ( fedge.f1 != cf && fedge.f1 != pf )
                {
                    int nf = fedge.f1 ;
                    pf = cf ;
                    cf = nf ;
                    continue ;
                }
                else  if ( fedge.f2 != cf && fedge.f2 != pf )
                {
                    int nf = fedge.f2 ;
                    pf = cf ;
                    cf = nf ;
                    continue ;
                }

            }
        }


        int end_face = cf ;

        if ( !isCycle   ) // not a cycle, find junctions
        {
            std::map<int, int>::const_iterator ji = junctions.find(start_face) ;

            if ( ji != junctions.end() ) start_junction = (*ji).second ;

            ji = junctions.find(end_face) ;

            if ( ji != junctions.end() ) end_junction = (*ji).second ;

        }

        // connect points into a chain

        int cv, pv = -1, nv ;

        if ( !isCycle && start_junction >= 0 ) chain.push_back(start_junction) ;

        for(int i=0 ; i<fchain.size()-1 ; i++ )
        {


            int face_idx = fchain[i] ;
            eFace &face = femap[face_idx] ;

            int v1 = -1, v2 = -1 ;
            if ( face.e1 != -1 ) v1 = binfo.edges[face.e1].v ;
            if ( face.e2 != -1 ) v2 = binfo.edges[face.e2].v ;

            int nv = -1 ;
            if ( v1 > 0 && v1 != pv && v1 != cv ) nv = v1 ;
            else if ( v2 > 0 && v2 != pv && v2 != cv ) nv = v2 ;

            if ( pv == -1 ) pv = nv ;
            else pv = cv ;

            cv = nv ;


            chain.push_back(cv) ;


        }

        if ( !isCycle ) {
            if ( end_junction >= 0 ) chain.push_back(end_junction) ;
            chain.push_back(-1) ; // marker for end of chain
        }

    }


}



struct vRegion {
public:

    unsigned short label ;

    std::set<int> vtxIdxs ; // vertices belonging to the region
    std::set<int> faces ; // faces belonging to the region
    std::set<unsigned short> adj ; // contains  adjacent regions

};

// compute the cost of merging two regions by fitting a quadratic surface over them and test whether the intermediate patch does fit

/*
static double computeMergeCost(const vRegion &r1, const vRegion &r2, const BoundaryInfo &binfo, const TriangleMesh2D &mesh, const TriangleMeshTopology &topo,
                               const cv::Mat_<ushort> &dim, const cv::Mat_<float> &gim)
{

    std::vector<Point> pts ;
    std::vector<double> Z ;

    std::set<int>::const_iterator fi = r1.faces.begin() ;

    for( ; fi != r1.faces.end() ; ++fi )
    {
        int fidx = (*fi) ;

        const TriangleMesh2D::Face &face = mesh.faces[fidx] ;

        Triangle2D tr(mesh.coords[face.v1], mesh.coords[face.v2], mesh.coords[face.v3]) ;
        TriangleScanIterator it(tr) ;


        while ( it )
        {
            Point p = *it ;

            unsigned short z = dim[p.y][p.x] ;

            if ( z != 0 )
            {
                pts.push_back(p) ;
                Z.push_back(z) ;
            }
            ++it ;
        }

    }


    fi = r2.faces.begin() ;

    for( ; fi != r2.faces.end() ; ++fi )
    {
        int fidx = (*fi) ;

        const TriangleMesh2D::Face &face = mesh.faces[fidx] ;

        TriangleScanIterator it(Triangle2D(mesh.coords[face.v1], mesh.coords[face.v2], mesh.coords[face.v3])) ;

        while ( it )
        {
            Point p = *it ;

            unsigned short z = dim[p.y][p.x] ;

            if ( z != 0 )
            {
                pts.push_back(p) ;
                Z.push_back(z) ;
            }
            ++it ;
        }
    }

    fi = binfo.faces.begin() ;

    double gcost = 0.0 ;
    int count = 0 ;

    for( ; fi != binfo.faces.end() ; ++fi )
    {
        int fidx = (*fi) ;

        const TriangleMesh2D::Face &face = mesh.faces[fidx] ;

        TriangleScanIterator it(Triangle2D(mesh.coords[face.v1], mesh.coords[face.v2], mesh.coords[face.v3])) ;

        while ( it )
        {
            Point p = *it ;

            unsigned short z = dim[p.y][p.x] ;

            gcost += gim[p.y][p.x] ;
            count ++ ;

            if ( z != 0 )
            {
                pts.push_back(p) ;
                Z.push_back(z) ;


            }
            ++it ;

        }
    }

    gcost /= count ;


    int ksize = pts.size() ;

    double *sx  = new double [ksize] ;
    double *sy  = new double [ksize] ;
    double *sxx = new double [ksize] ;
    double *syy = new double [ksize] ;
    double *sxy = new double [ksize] ;
    double *zz  = new double [ksize] ;

    for( int i=0 ; i<ksize ; i++ )
    {
        double ux = pts[i].x/1000.0; ;
        double uy = pts[i].y/1000.0 ;
        sy[i] = uy ;  sx[i] = ux ;
        sxx[i] = ux*ux ; syy[i] = uy*uy ;
        sxy[i] = ux*uy ;
        zz[i] = Z[i]/10000.0 ;

    }

    cv::Mat_<double> A(6, 6) ;

    double s11 = 0, s12 = 0, s13 = 0, s14 = 0, s15 = 0, s16 = 0 ;
    double s22 = 0, s23 = 0, s24 = 0, s25 = 0, s26 = 0 ;
    double s33 = 0, s34 = 0, s35 = 0, s36 = 0 ;
    double s44 = 0, s45 = 0, s46 = 0 ;
    double s55 = 0, s56 = 0, s66 = 0 ;
    double q1 = 0, q2 = 0, q3 = 0, q4 = 0, q5 = 0, q6 = 0 ;

    for( int i=0 ; i<ksize ; i++ )
    {
        double pu = sx[i],   pv = sy[i] ;
        double p2u = sxx[i], p2v = syy[i] ;
        double puv = sxy[i], pz = zz[i] ;
        double p4u = p2u * p2u, p4v = p2v * p2v ;
        double p3uv = p2u * puv, pu3v = puv * p2v ;
        double p2u2v = puv * puv, p2uv = p2u * pv, pu2v = pu * p2v ;
        double p3u = p2u * pu, p3v = p2v * pv ;

        //[     u^4, u^2*v^2,   u^3*v,     u^3,   u^2*v,     u^2]
        s11 += p4u ; s12 += p2u2v ; s13 += p3uv ; s14 += p3u ; s15 += p2uv ; s16 += p2u ;
        //[ u^2*v^2,     v^4,   v^3*u,   v^2*u,     v^3,     v^2]
        s22 += p4v ; s23 += pu3v ; s24 += pu2v ; s25 += p3v ; s26 += p2v ;
        //[   u^3*v,   v^3*u, u^2*v^2,   u^2*v,   v^2*u,     u*v]
        s33 += p2u2v ; s34 += p2uv; s35 += pu2v; s36 += puv ;
        //[     u^3,   v^2*u,   u^2*v,     u^2,     u*v,       u]
        s44 += p2u ; s45 += puv ; s46 += pu ;
        //[   u^2*v,     v^3,   v^2*u,     u*v,     v^2,       v]
        s55 += p2v ; s56 += pv ;
        //[     u^2,     v^2,     u*v,       u,       v,       1]
        s66 += 1.0 ;
        //u^2 b,   v^2 b,  u v b, u b, v b, b
        q1 += p2u * pz ; q2 += p2v * pz ; q3 += puv * pz ;
        q4 += pu * pz ; q5 += pv * pz ; q6 += pz ;
    }


    A(0,0) = s11 ; A(0,1) = s12 ; A(0,2) = s13 ; A(0,3) = s14 ; A(0,4) = s15 ; A(0,5) = s16 ;
    A(1,0) = s12 ; A(1,1) = s22 ; A(1,2) = s23 ; A(1,3) = s24 ; A(1,4) = s25 ; A(1,5) = s26 ;
    A(2,0) = s13 ; A(2,1) = s23 ; A(2,2) = s33 ; A(2,3) = s34 ; A(2,4) = s35 ; A(2,5) = s36 ;
    A(3,0) = s14 ; A(3,1) = s24 ; A(3,2) = s34 ; A(3,3) = s44 ; A(3,4) = s45 ; A(3,5) = s46 ;
    A(4,0) = s15 ; A(4,1) = s25 ; A(4,2) = s35 ; A(4,3) = s45 ; A(4,4) = s55 ; A(4,5) = s56 ;
    A(5,0) = s16 ; A(5,1) = s26 ; A(5,2) = s36 ; A(5,3) = s46 ; A(5,4) = s56 ; A(5,5) = s66 ;


    delete [] sx ; delete [] sy ; delete [] zz ;
    delete [] sxx ; delete [] syy ; delete [] sxy ;

    cv::Mat_<double> B(6, 1), X ;

    B(0, 0) = q1 ; B(1, 0) = q2 ; B(2, 0) = q3 ; B(3, 0) = q4 ; B(4, 0) = q5 ; B(5, 0) = q6 ;

    cv::solve(A, B, X) ;

    double a = X(0, 0), b = X(1, 0), c = X(2, 0), d = X(3, 0), e = X(4, 0), f = X(5, 0) ;

    double denom = 4*a*b -c*c ;

    double cost = 0 ;
    int ncount= 0 ;

    for( int i=0 ; i<ksize ; i++ )
    {
        double x0 = pts[i].x/1000.0 ;
        double y0 = pts[i].y/1000.0 ;

        double _Z = a * x0 * x0 + b * y0 * y0 + c *x0 * y0 + d * x0 + e * y0 + f ;

        cost += fabs(_Z *10000 - Z[i]) ;

    }

    cost /= ksize ;

   return cost + gcost/20 ;


}
*/


void seg(const cv::Mat &dim_, const cv::Mat &cim_)
{
    int w = dim_.cols, h = dim_.rows ;

    cv::Mat_<ushort> dim(dim_) ;
    // triangulate image domain using Delaunay mesh

    Polygon2D poly ;
    poly.addPoint(Point2D(0, 0)) ;
    poly.addPoint(Point2D(w-1, 0)) ;
    poly.addPoint(Point2D(w-1, h-1)) ;
    poly.addPoint(Point2D(0, h-1)) ;

    TriangleMesh2D mesh ;

    const double triangleArea = w*h/1000 ;

    triangulate(poly, triangleArea, mesh) ;

    // iterate over the edges of the triangulation and compute their cost

    TriangleMeshTopology topo ;
    mesh.getTopology(topo) ;

    cv::Mat cim_blurred, mag, ang ;

    const double gradThreshold = 10.0 ;

    cv::blur(cim_, cim_blurred, cv::Size(3, 3)) ;
    computeGradient(cim_blurred, mag, ang, gradThreshold);

    int nEdges = topo.getNumEdges() ;

    edge *edges = new edge [nEdges] ;

    EdgeIterator it(topo) ;

    int count = 0 ;

    cv::Mat cc(h, w, CV_8UC1) ;

    cc = cv::Scalar(0) ;

    cv::Mat clr__ = cim_.clone() ;

    while ( it )
    {

        int v1 = (*it).first ;
        int v2 = (*it).second ;

        double cost = computeEdgeCost(mag, dim_, mesh.coords[v1], mesh.coords[v2]) ;

        edge &e = edges[count++] ;
        e.a = v1 ;
        e.b = v2 ;
        e.w = cost ;

        cv::line(clr__,  cv::Point(mesh.coords[v1].x(), mesh.coords[v1].y()), cv::Point(mesh.coords[v2].x(), mesh.coords[v2].y()), cv::Scalar(0, 255, 0)) ;
        ++it ;
    }

    cv::imwrite("/tmp/edges.png", clr__) ;

    const float cParam = 0.1 ;
    const int minComponentSize = 5;

    disjoint_set_forest *u = segment_graph(mesh.coords.size(), nEdges, edges, cParam) ;


    // post process small components
     for (int i = 0; i < nEdges; i++) {
        int a = u->find(edges[i].a);
        int b = u->find(edges[i].b);
        if ((a != b) && ((u->size(a) < minComponentSize) || (u->size(b) < minComponentSize)))
            u->join(a, b);
        }

     int num_ccs = u->num_sets();

      std::set<unsigned short> comp ; // components
      std::vector<unsigned short> labels ; // vertex labels

    // collect resulting labels

     for(int i=0 ; i<mesh.coords.size() ; i++)
     {
        int c = u->find(i) ;
        comp.insert(c) ;
        labels.push_back(c) ;
     }

     // build the network of region boundaries

     std::vector<Point2D> ivtx ;

     typedef std::map< std::pair<int, int>, BoundaryInfo > BoundaryList ;

     BoundaryList boundaries ; // region boundaries

     for (int i = 0; i < nEdges; i++)
     {
         int v1 = edges[i].a ;
         int v2 = edges[i].b ;

         int c1 = u->find(v1);
         int c2 = u->find(v2);

         if ( c1 != c2 )
         {
             std::pair<int, int> boundary(std::min(c1, c2), std::max(c1, c2)) ;

             int f1, f2 ;
             topo.edgeGetAdjFaces(v1, v2, f1, f2) ;

             BoundaryInfo &info = boundaries[boundary] ;

             cEdge edge ;

             Point2D pc =  0.5*(mesh.coords[v1] + mesh.coords[v2]) ;

             ivtx.push_back(pc) ;

             int v = ivtx.size() - 1 ;

             edge.f1 = f1 ;
             edge.f2 = f2 ;
             edge.v = v ;

             info.edges.push_back(edge) ;

        }
    }

     // detect faces/junctions

     std::map<int, int> junctions ; // maps faces to associated junction points

     for(int i=0 ; i<mesh.faces.size() ; i++ )
     {

         TriangleMesh2D::Face &face = mesh.faces[i] ;

         int v1 = face.v1 ;
         int v2 = face.v2 ;
         int v3 = face.v3 ;

         int c1 = labels[v1];
         int c2 = labels[v2];
         int c3 = labels[v3];

         if ( c1 != c2 && c2 != c3 )
         {
             Point2D pc =  (mesh.coords[v1] + mesh.coords[v2] + mesh.coords[v3])/3 ;

             ivtx.push_back(pc) ;

             int v = ivtx.size() - 1 ;

             junctions[i] = v ;
         }
    }

     BoundaryList::const_iterator eit = boundaries.begin() ;

     cv::Mat ccc = cim_.clone() ;

     int k=0 ;

     while ( eit != boundaries.end() )
     {

         std::pair<int, int> key = (*eit).first ;

         std::vector<int> chain ;


         makeChain((*eit).second, junctions, chain) ;

         for(int i=0 ; i<chain.size() ; i++ )
         {
             int v1 = chain[i] ;
             int v2 = chain[i+1] ;

             if ( v1 != -1 && v2 != -1 )
             {
                const Point2D &p1 = ivtx[v1] ;
                const Point2D &p2 = ivtx[v2] ;

                cv::line(ccc, cv::Point(p1.x(), p1.y()), cv::Point(p2.x(), p2.y()), cv::Scalar(255, 255, 255) ) ;

                cout << p1 << ' ' << p2 << ' ' << k << endl ;
                if ( p2.y() > 470 )
                {
                    cout << "ok here" << endl ;
                }
             }

         }

         ++k ;
         ++eit ;


     }



    cv::imwrite("/home/malasiot/tmp/edges.png", ccc) ;




/*
    std::vector<cv::Vec3b> colors ;

     for(int i=0 ; i<mesh.coords.size() ; i++)
     {
         unsigned char r = rand()%255 ;
         unsigned char g = rand()%255 ;
         unsigned char b = rand()%255 ;

         colors.push_back(cv::Vec3b(r, g, b)) ;
     }

     cv::Mat_<cv::Vec3b> cim(cim_) ;

     for(int i=0 ; i<mesh.coords.size() ; i++ )
     {
         int comp = u->find(i);

         cv::circle(cim, cv::Point(mesh.coords[i].x, mesh.coords[i].y), 2, cv::Scalar(colors[comp][0], colors[comp][1], colors[comp][2]), -1) ;


     }

     for (int i = 0; i < nEdges; i++)
     {
         int v1 = edges[i].a ;
         int v2 = edges[i].b ;

         int c1 = u->find(v1);
         int c2 = u->find(v2);

        if ( c1 != c2 )
             cv::line(cc, cv::Point(mesh.coords[v1].x, mesh.coords[v1].y), cv::Point(mesh.coords[v2].x, mesh.coords[v2].y), cv::Scalar(255) ) ;

    }


     cv::imwrite("/home/malasiot/tmp/edges.png", cc) ;

    cv::imwrite("/home/malasiot/tmp/labels.png", cim) ;

*/
/*
    // create label image

    cv::Mat_<ushort> labels(h, w) ;

    labels = ushort(0) ;

    for(int i=0 ; i<mesh.faces.size() ; i++ )
    {

        TriangleMesh2D::Face &face = mesh.faces[i] ;

        int v1 = face.v1 ;
        int v2 = face.v2 ;
        int v3 = face.v3 ;

        int c1 = comp[v1];
        int c2 = comp[v2];
        int c3 = comp[v3];

   //     cout << c1 << ' ' << c2 << ' ' << c3 << endl ;

        if ( c1 == c2 && c2 == c3 )
            scanTriangle(labels, dim, c1, mesh.coords[v1], mesh.coords[v2], mesh.coords[v3]) ;

        else if ( c1 == c2 && c2 != c3)
        {
            Point2D pc1 = 0.5*(mesh.coords[v1] + mesh.coords[v3]) ;
            Point2D pc2 = 0.5*(mesh.coords[v2] + mesh.coords[v3]) ;

            scanTriangle(labels, dim, c1, mesh.coords[v1], pc1, mesh.coords[v2]) ;
            scanTriangle(labels, dim, c1, mesh.coords[v2], pc1, pc2) ;
            scanTriangle(labels, dim, c3, pc1, mesh.coords[v3], pc2) ;
        }
        else if ( c1 == c3 && c2 != c3)
        {
            Point2D pc1 = 0.5*(mesh.coords[v1] + mesh.coords[v2]) ;
            Point2D pc2 = 0.5*(mesh.coords[v2] + mesh.coords[v3]) ;

            scanTriangle(labels, dim, c1, mesh.coords[v1], pc1, mesh.coords[v3]) ;
            scanTriangle(labels, dim, c1, mesh.coords[v3], pc1, pc2) ;
            scanTriangle(labels, dim, c2, pc1, mesh.coords[v2], pc2) ;
        }
        else if ( c2 == c3 && c2 != c1)
        {
            Point2D pc1 = 0.5*(mesh.coords[v1] + mesh.coords[v2]) ;
            Point2D pc2 = 0.5*(mesh.coords[v1] + mesh.coords[v3]) ;

            scanTriangle(labels, dim, c2, mesh.coords[v2], pc1, mesh.coords[v3]) ;
            scanTriangle(labels, dim, c2, mesh.coords[v3], pc1, pc2) ;
            scanTriangle(labels, dim, c1, pc1, mesh.coords[v1], pc2) ;
        }
        else {

            scanTriangle(labels, dim, 0, mesh.coords[v1], mesh.coords[v2], mesh.coords[v3]) ;

        }


    }
*/

    // collect region info
/*
     std::map<unsigned short, vRegion> regions ;

     typedef std::map< std::pair<int, int>, BoundaryInfo > BoundaryList ;
     BoundaryList boundaries ; // region boundaries

     for (int i = 0; i < nEdges; i++)
     {
         int v1 = edges[i].a ;
         int v2 = edges[i].b ;

         int c1 = u->find(v1);
         int c2 = u->find(v2);

         if ( c1 == c2 )
         {
             vRegion &reg = regions[c1] ;
             reg.label = c1 ;

             reg.vtxIdxs.insert(v1) ;
             reg.vtxIdxs.insert(v2) ;
         }
         else
         {
             std::pair<int, int> boundary(std::min(c1, c2), std::max(c1, c2)) ;

             int f1, f2 ;
             topo.edgeGetAdjFaces(v1, v2, f1, f2) ;

             BoundaryInfo &info = boundaries[boundary] ;

             info.edges.insert(i) ;
             info.faces.insert(f1) ;
             info.faces.insert(f2) ;

             vRegion &reg1 = regions[c1] ;
             reg1.label = c1 ;
             reg1.vtxIdxs.insert(v1) ;
             reg1.adj.insert(c2) ;

             vRegion &reg2 = regions[c2] ;
             reg2.label = c2 ;
             reg2.vtxIdxs.insert(v2) ;
             reg2.adj.insert(c1) ;

        }
    }

     for(int i=0 ; i<mesh.faces.size() ; i++ )
     {
          TriangleMesh2D::Face &face = mesh.faces[i] ;

         int v1 = face.v1 ;
         int v2 = face.v2 ;
         int v3 = face.v3 ;

         int c1 = labels[v1];
         int c2 = labels[v2];
         int c3 = labels[v3];

         if ( c1 == c2 && c2 == c3 )
         {
             vRegion &reg = regions[c1] ;
             reg.faces.insert(i) ;
         }
    }

     std::map<unsigned short, int> regMap ;
     std::vector<unsigned short> regList ;

     std::map<unsigned short, vRegion>::iterator rit = regions.begin() ;

     while ( rit != regions.end() )
     {

         regList.push_back((*rit).first) ;
         regMap[(*rit).first] = regList.size() -1 ;
         ++rit ;
     }

     // compute boundary (edge) cost

     BoundaryList::iterator bit = boundaries.begin();

    for( ; bit != boundaries.end() ; ++bit )
    {
        const std::pair<int, int> &lbpair = (*bit).first ;
        unsigned short r1 = lbpair.first ;
        unsigned short r2 = lbpair.second ;

        BoundaryInfo &binfo = (*bit).second ;

        double cost = computeMergeCost(regions[r1], regions[r2], binfo, mesh, topo, dim, mag ) ;
        binfo.cost = cost ;



    }

    // now perform graph segmentation again but this time vertices are the regions and edges are their boundaries

    delete [] edges ;
    delete u ;

    edges = new edge [boundaries.size()] ;

    bit = boundaries.begin();

    count = 0 ;

    for( ; bit != boundaries.end() ; ++bit )
    {
       const std::pair<int, int> &lbpair = (*bit).first ;
       unsigned short r1 = lbpair.first ;
       unsigned short r2 = lbpair.second ;

       edges[count].a = regMap[r1] ;
       edges[count].b = regMap[r2] ;
       edges[count].w = (*bit).second.cost ;
       ++count ;

   }

    nEdges = count ;

    const float rcParam = 5.5 ;

    u = segment_graph(regList.size(), count, edges, rcParam) ;

    num_ccs = u->num_sets();


    typedef std::set<unsigned short> SuperRegion ;

    std::map<unsigned short, SuperRegion> sregions ;

    for (int i = 0; i < nEdges; i++)
    {
        int v1 = edges[i].a ;
        int v2 = edges[i].b ;

        int c1 = u->find(v1);
        int c2 = u->find(v2);

        unsigned short r1 = regList[v1] ;
        unsigned short r2 = regList[v2] ;

        if ( c1 == c2 )
        {
            SuperRegion &sr = sregions[c1] ;
            sr.insert(r1) ;
            sr.insert(r2) ;
        }
        else
        {
            SuperRegion &sr = sregions[c1] ;
            sr.insert(r1) ;

            sr = sregions[c2] ;
            sr.insert(r2) ;
        }

   }

    cv::Mat_<ushort> labels_(h, w) ;
    labels_ = (ushort)0 ;

    for( int i=0 ; i<regList.size() ; i++ )
    {
        unsigned short lab = regList[i] ;

        int c = u->find(i) ;

        const vRegion &reg = regions[lab] ;

        std::set<int>::const_iterator reg_it = reg.faces.begin() ;

        for( ; reg_it != reg.faces.end() ; reg_it ++ )
        {
             int face_idx = (*reg_it) ;
             TriangleMesh2D::Face &face = mesh.faces[face_idx] ;

             scanTriangle(labels_, dim, c, mesh.coords[face.v1], mesh.coords[face.v2], mesh.coords[face.v3]) ;

         }

    }

 cv::imwrite("/home/malasiot/tmp/elabels.png", labels_) ;

    delete  u ;

*/



/*


     EdgeMap emap ;

     std::set< std::pair<int, int> > boundaries ;

    std::set< std::pair<int, int> > boundaries ;

    std::vector<Point2D> ivtx ;

    EdgeMap emap ;

    std::set< std::pair<int, int> > boundaries ;
    std::set< unsigned short > labels ;

    for (int i = 0; i < nEdges; i++)
    {
        int v1 = edges[i].a ;
        int v2 = edges[i].b ;

        int c1 = u->find(v1);
        int c2 = u->find(v2);

        if ( c1 != c2 )
        {
            std::pair<int, int> boundary(std::min(c1, c2), std::max(c1, c2)) ;

            boundaries.insert(boundary) ;

            labels.insert(c1) ;
            labels.insert(c2) ;

            cEdge edge ;

            Point2D pc =  0.5*(mesh.coords[v1] + mesh.coords[v2]) ;

            ivtx.push_back(pc) ;
            int v = ivtx.size() - 1 ;

            int f1, f2 ;
            topo.edgeGetAdjFaces(v1, v2, f1, f2) ;

            edge.f1 = f1 ;
            edge.f2 = f2 ;
            edge.v = v ;

            emap.insert(EdgeMap::value_type(boundary, edge));

            cout << c1 << ' ' << c2 << endl ;

           //cv::line(labels, cv::Point(mesh.coords[v1].x, mesh.coords[v1].y), cv::Point(mesh.coords[v2].x, mesh.coords[v2].y), cv::Scalar(255) ) ;
       }
   }

    cv::Mat_<ushort> elabels(h, w) ;

    elabels = ushort(0) ;

    // iterate over all recorded boundaries

    std::set< std::pair<int, int> >::const_iterator eit = boundaries.begin() ;

    ushort elabel = 1 ;

    while ( eit != boundaries.end() )
    {

        std::pair<int, int> key = (*eit) ;

        std::pair<EdgeMap::const_iterator, EdgeMap::const_iterator> range = emap.equal_range(key) ;

         cout << key.first << ' ' << key.second << ' ' << emap.count(key) << endl ;

        std::vector<int> chain ;
        makeChain(range, chain) ;

        for(int i=0 ; i<chain.size()-1 ; i++ )
        {
            int v1 = chain[i] ;
            int v2 = chain[i+1] ;

            const Point2D &p1 = ivtx[v1] ;
            const Point2D &p2 = ivtx[v2] ;

            cv::line(elabels, cv::Point(p1.x, p1.y), cv::Point(p2.x, p2.y), cv::Scalar(elabel) ) ;

        }



        elabel ++ ;


        ++eit ;


    }


    cv::imwrite("/home/malasiot/tmp/elabels.png", elabels) ;


   //  cv::imwrite("/home/malasiot/tmp/labels2.png", labels) ;
*/

}

