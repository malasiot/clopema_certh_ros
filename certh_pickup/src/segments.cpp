#include <cv.h>
#include <highgui.h>

#include <certh_libs/Line2D.h>
#include <certh_libs/cvHelpers.h>
#include <certh_libs/TriangleMesh2D.h>


using namespace std ;
using namespace certh_libs ;

////////////////////////////////////////////////////////////////////////

void samplePoints(const vector<cv::Point2d> &in_pts, vector<Point2D> &out_pts, double lStep)
{
    double plen = 0.0 ;

    int c = 0 ;
    Point2D cp(in_pts[0].x, in_pts[0].y) ;

    out_pts.push_back(cp) ;

    do
    {
        Point2D np(in_pts[c+1].x, in_pts[c+1].y) ;

        double slen = ( cp - np).norm() ;

        if ( slen < lStep ) {
            cp = np ;
            ++c ;
        }
        else
        {
            double h = lStep/slen ;
            cp = cp * (1-h) + np * h ;
            out_pts.push_back(cp) ;
        }

    }  while ( c < in_pts.size() - 1 ) ;

}

void detectEdgeSegments(const cv::Mat &cim_, const cv::Mat &dim_,  vector< vector<Point2D> > &scontour  )
{
    int w = dim_.cols, h = dim_.rows ;
    cv::Mat gim ;
    cv::Mat_<uchar> mask ;

    // convert the image to grayscale

    cv::Mat_<ushort> dim(dim_.clone()) ;
    cv::Mat_<cv::Vec3b> cim(cim_) ;

    mask = (uchar)0 ;

    double minZ;
    double maxZ;
    cv::minMaxIdx(dim_, &minZ, &maxZ);

    cv::Mat adjMap ;
    cv::convertScaleAbs(dim_, adjMap, 255 / maxZ);

    dim.convertTo(gim, CV_8UC1) ;

    /// Reduce noise with a kernel 3x3
    cv::blur( gim, gim, cv::Size(3,3) );

    // perform Canny edge detection

    cv::Mat edges ;
    cv::Canny(gim, edges, 20, 40, 3) ;

    // find contours

    vector< vector<cv::Point> > contours ;

    edgeLinking(edges, contours) ;

    // break contours into linear segments

    for(int i=0 ; i<contours.size() ; i++)
    {
        if ( contours[i].size() < 10 ) continue ;

        vector<cv::Point2d> dc, sc ;
        vector<Point2D> oc ;

        for(int j=0 ; j<contours[i].size() ; j++)
            dc.push_back(cv::Point2d(contours[i][j].x, contours[i][j].y)) ;

        simplifyContour(dc, sc, 2.0);

        samplePoints(sc, oc, 7.0) ;

        scontour.push_back(oc) ;

        for(int j=0 ; j<oc.size()-1 ; j++)
        {
            cv::circle(cim, cv::Point(oc[j].x(), oc[j].y()), 1.5, cv::Scalar(255, 0, 0)) ;
        }
    }

    cv::imwrite("/home/malasiot/tmp/edges.png", cim) ;
}


void findInitialCandidates(const cv::Mat &cim_, const cv::Mat &dim_, vector<Point2D> &candidates)
{
    int w = dim_.cols, h = dim_.rows ;

    cv::Mat_<ushort> dim(dim_) ;
    // triangulate image domain using Delaunay mesh

    Polygon2D poly ;

    vector< vector<Point2D> > segs_ ;
    detectEdgeSegments(cim_, dim_, segs_) ;

    vector<Point2D> allPts ;

    for(int i=0 ; i<segs_.size() ; i++ )
        for(int j=0 ; j<segs_[i].size() ; j++ )
            allPts.push_back(segs_[i][j]) ;

    convexHull(allPts, poly) ;

    vector< vector< Point2D > > segs ;

    TriangleMesh2D mesh ;

    const double triangleArea = 85 ;

    triangulate(poly, segs, triangleArea, mesh) ;

    int nPts = mesh.coords.size() ;

    candidates = mesh.coords ;

    cv::Mat cim(cim_.clone()) ;

    for (int i=0 ; i<nPts ; i++)
    {
        const Point2D &p = mesh.coords[i] ;

        cv::circle(cim, cv::Point(p.x(), p.y()), 1.5, cv::Scalar(255, 0, 0)) ;
    }

    cv::imwrite("/home/malasiot/tmp/mesh.png", cim) ;

}
