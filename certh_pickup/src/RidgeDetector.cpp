#include "RidgeDetector.h"

#include <cv.h>
#include <highgui.h>

#include <certh_libs/Line2D.h>
#include <certh_libs/cvHelpers.h>
#include <certh_libs/TriangleMesh2D.h>
#include <certh_libs/PolygonScanner.h>

#include <Eigen/SVD>
#include <Eigen/Eigenvalues>

using namespace std ;
using namespace certh_libs ;
using namespace cv ;
using namespace Eigen ;

static void nonMaximaSuppressionDirectional(const Mat& src, Mat& dst, const Mat &alpha_, const Mat &sigma_)
{
    int w = src.cols, h = src.rows ;

    dst = Mat(h, w, CV_8UC1) ;

    for( int i=0 ; i<h ; i++ )
        for( int j=0 ; j<w ; j++ )
        {
            double alpha = alpha_.at<float>(i, j) ;
            double sigma = sigma_.at<float>(i, j) ;

            double sa = cos(alpha) ;
            double ca = -sin(alpha) ;

            int ppx = j + sigma *  ca + 0.5;
            int ppy = i + sigma *  sa + 0.5 ;

            int pnx = j - sigma *  ca + 0.5;
            int pny = i - sigma *  sa + 0.5 ;

            vector<cv::Point> pts ;

            getScanLine(cv::Point(pnx, pny), cv::Point(ppx, ppy), pts) ;

            float maxV = 0.0 ;
            cv::Point mp ;

            for( int k=0 ; k<pts.size() ; k++ )
            {
                float v = src.at<float>(pts[k]) ;

                if ( v > maxV )
                {
                    maxV = v ;
                    mp = pts[k] ;
                }
             }

            if ( i == mp.y && j == mp.x ) dst.at<uchar>(i, j) = 255 ;
            else dst.at<uchar>(i, j) = 0 ;
        }
}


static double graspable(const cv::Mat &src, double x, double y, double alpha, double sigma, double gripperOpenning, double gripperWidth)
{
    Vector2 p0(x, y) ;

    double sa = cos(alpha) ;
    double ca = -sin(alpha) ;

    Vector2 d(ca, sa) ;
    Vector2 n(sa, -ca) ;

    // find all points that are withing the gripper's profile (openned)

    Polygon2D poly ;

    poly.addPoint(p0 + d * gripperWidth/2.0 + n * gripperOpenning/2.0) ;
    poly.addPoint(p0 + d * gripperWidth/2.0 - n * gripperOpenning/2.0) ;
    poly.addPoint(p0 - d * gripperWidth/2.0 - n * gripperOpenning/2.0) ;
    poly.addPoint(p0 - d * gripperWidth/2.0 + n * gripperOpenning/2.0) ;

    vector<certh_libs::Point> pts ;
    getPointsInPoly(poly, pts);

    // project points to the center line

    vector<double> X, Y ;

    int nc = 0 ;
    double area = 0 ;
    double offset = 0 ;

    for(int i=0 ; i<pts.size() ; i++ )
    {
        const certh_libs::Point &p = pts[i] ;

        int x_ = p.x() ;
        int y_ = p.y() ;

        if ( x_<0 || y_<0 || x_ >= src.cols-1 || y_ >= src.rows-1 ) continue ;

        float val = src.at<float>(y_, x_) ;

        double ds = Vector2(x_ - x, y_ - y).dot(d) ;

        if ( fabs(ds) > gripperOpenning/4 )
        {
            offset = std::max(offset, (double)val) ;
        }
        else
        {
            area += val ;
            nc ++ ;
        }

    }

    area /= nc ;
    //area -= offset * gripperWidth/2 ;
    area -= offset ;

    return area ;

}

Mat RidgeDetector::detect(const Mat &src, Mat &alpha, Mat &scale, Mat &ridges)
{
    Mat gray ;
    src.convertTo(gray, CV_32FC1) ;

    int w = src.cols, h = src.rows ;

    double responseThreshold = params.responseThreshold ;

    Mat resp(h, w, CV_32FC1, Scalar(0)) ;
    alpha = cv::Mat(h, w, CV_32FC1, Scalar(0)) ;
    scale = cv::Mat(h, w, CV_32FC1, Scalar(0)) ;

    for( double sigma = params.minScale ; sigma <= params.maxScale ; sigma += params.scaleStep )
    {
        Mat blured ;

        GaussianBlur(gray, blured, Size(0, 0), sigma) ;

        Mat gxy, g2x, g2y, gx, gy;

        Sobel(blured, gxy, CV_32FC1, 1, 1) ;
        Sobel(blured, g2x, CV_32FC1, 2, 0) ;
        Sobel(blured, g2y, CV_32FC1, 0, 2) ;

        Sobel(blured, gx, CV_32FC1, 1, 0) ;
        Sobel(blured, gy, CV_32FC1, 0, 1) ;

        for( int i=0 ; i<h ; i++ )
            for( int j=0 ; j<w ; j++ )
            {
                // find the orinetation of the ridge

                float gxy_ = gxy.at<float>(i, j) ;
                float g2x_ = g2x.at<float>(i, j) ;
                float g2y_ = g2y.at<float>(i, j) ;
                float gx_ = gx.at<float>(i, j) ;
                float gy_ = gy.at<float>(i, j) ;

                float gd = g2x_ - g2y_ ;

                if ( gx_ * gx_ + gy_ * gy_ < 5 * 5 ) continue ;

                double a = atan2(2*gxy_, gd)/2 ;

                double sa = cos(a) ;
                double ca = -sin(a) ;

                int ppx = j + sigma *  ca + 0.5;
                int ppy = i + sigma *  sa + 0.5 ;

                int pnx = j - sigma *  ca + 0.5;
                int pny = i - sigma *  sa + 0.5 ;

                if ( ppx < 0 || ppy < 0 || ppx >= w-1 || ppy >= h-1 ) continue ;
                if ( pnx < 0 || pny < 0 || pnx >= w-1 || pny >= h-1 ) continue ;

                double rl = gx.at<float>(ppy, ppx) * ca + gy.at<float>(ppy, ppx) * sa ;
                double rr = gx.at<float>(pny, pnx) * ca + gy.at<float>(pny, pnx) * sa ;

                rl = std::max(-rl, 0.0) ;
                rr = std::max(rr, 0.0) ;

                double resp_ = std::min(rl, rr) ;

                if ( resp_ < responseThreshold ) continue ;

                if ( resp_ > resp.at<float>(i, j) )
                {
                    resp.at<float>(i, j) = resp_ ;
                    alpha.at<float>(i, j) = a ;
                    scale.at<float>(i, j) = sigma ;
                }

            }

    }

    nonMaximaSuppressionDirectional(resp, ridges, alpha, scale);

    return resp ;
}

struct CandSorter {
    bool operator() (const RidgeDetector::GraspCandidate &c1, const RidgeDetector::GraspCandidate &c2)
    {
        return c1.strength >= c2.strength ;
    }
};

void RidgeDetector::findCandidates(const Mat &src, const Mat &resp, const Mat &ridges, const Mat &alpha, const Mat &scale, std::vector<GraspCandidate> &candList)
{
    int w = src.cols, h = src.rows ;

    Mat gray ;
    src.convertTo(gray, CV_32FC1) ;

    deque<GraspCandidate> cand ;

    for( int i=0 ; i<h ; i++ )
        for(int j=0 ; j<w ; j++ )
        {
            if ( ridges.at<uchar>(i, j) == 0 ) continue ;

            GraspCandidate grp ;
            grp.x = j ;
            grp.y = i ;
            grp.strength = resp.at<float>(i, j) ;
            grp.width = scale.at<float>(i, j) ;
            grp.alpha = alpha.at<float>(i, j) ;

            cand.push_back(grp) ;
        }

    sort(cand.begin(), cand.end(), CandSorter()) ;

    // remove neighbors of the best candidate that have similar orientations

    const double radius = 8 ;

    while ( !cand.empty() )
    {
        // insert best candidate in the list

        GraspCandidate best = cand.front() ;
        cand.pop_front() ;

        candList.push_back(best) ;

        double sa = -sin(best.alpha) ;
        double ca = cos(best.alpha) ;
        Vector2 d(sa, ca) ;

        deque<GraspCandidate>::iterator it = cand.begin() ;

        // check neighbors

        while ( it != cand.end() )
        {
            const GraspCandidate &cnd = *it ;

            if ( ( cnd.x - best.x ) * ( cnd.x - best.x ) + ( cnd.y - best.y ) * ( cnd.y - best.y ) > radius * radius) {
                ++it ;
                continue ;
            }

            if ( fabs(d.dot(Vector2(cnd.x - best.x, cnd.y - best.y ))) < best.width/2 &&
                      fabs(best.alpha - cnd.alpha) < 0.05 )
                it = cand.erase(it) ;
            else
                ++it ;
        }
    }


    return ;


}


bool RidgeDetector::detect(const cv::Mat &src, std::vector<GraspCandidate> &cand)
{
    Mat scale, alpha, ridges, resp ;

    resp = detect(src, alpha, scale, ridges) ;
    findCandidates(src, resp, ridges, alpha, scale, cand) ;

    return !cand.empty() ;

}


void RidgeDetector::draw(cv::Mat &clr, const std::vector<GraspCandidate> &cand)
{
    for(int i=0 ; i<cand.size() ; i++ )
    {
        const GraspCandidate &cnd = cand[i] ;

        double sa = -sin(cnd.alpha) ;
        double ca = cos(cnd.alpha) ;

        Vector2 d(sa, ca) ;

        Vector2 p1 =  d * cnd.width + Vector2(cnd.x, cnd.y) ;
        Vector2 p2 = -d * cnd.width + Vector2(cnd.x, cnd.y) ;

        if ( i==0 )
            cv::line(clr, cv::Point(p1.x(), p1.y()), cv::Point(p2.x(), p2.y()), Scalar(0, 255, 0), 2 )  ;
        else if ( i < 10 )
            cv::line(clr, cv::Point(p1.x(), p1.y()), cv::Point(p2.x(), p2.y()), Scalar(255, 255, 0), 1 )  ;
        else
            cv::line(clr, cv::Point(p1.x(), p1.y()), cv::Point(p2.x(), p2.y()), Scalar(255, 255, 255), 1 )  ;



    }
}
