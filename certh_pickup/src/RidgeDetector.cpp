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
            double ca = sin(alpha) ;

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
    double ca = sin(alpha) ;

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

void RidgeDetector::detect(const Mat &src, Mat &alpha, Mat &scale, Mat &ridges)
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
                float gd = g2x.at<float>(i, j) - g2y.at<float>(i, j) ;
                if ( fabs(gd) < 1.0e-5 ) continue ;

                double a = atan(2*gxy.at<float>(i, j)/gd)/2 ;

                double sa = cos(a) ;
                double ca = sin(a) ;

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
}

Mat RidgeDetector::computeGraspability(const Mat &src, const Mat &ridges, const Mat &alpha, const Mat &scale, double gripperOpenning, double gripperWidth)
{
    int w = src.cols, h = src.rows ;

    Mat gray ;
    src.convertTo(gray, CV_32FC1) ;

    Mat gsp(h, w, CV_32FC1, Scalar(0.0)) ;

    for( int i=0 ; i<h ; i++ )
        for(int j=0 ; j<w ; j++ )
        {
            if ( ridges.at<uchar>(i, j) == 0 ) continue ;

            float gsp_ = graspable(gray, j, i, alpha.at<float>(i, j), scale.at<float>(i, j), gripperOpenning, gripperWidth)  ;

            gsp.at<float>(i, j) = gsp_ ;
        }

    return gsp ;
}
