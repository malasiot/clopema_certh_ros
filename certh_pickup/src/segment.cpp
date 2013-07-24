#include "ObjectOnPlaneDetector.h"

#include <highgui.h>
#include <iostream>

extern void seg(const cv::Mat &dim_, const cv::Mat &cim_) ;

using namespace std;

int main(int argc, char *argv)
{

    cv::Mat clr = cv::imread("/home/malasiot/images/clothes/on_table/kinect_grab_000130_c.tif") ;
    cv::Mat depth = cv::imread("/home/malasiot/images/clothes/on_table/kinect_grab_000130_d.tif", -1) ;

    ObjectOnPlaneDetector det(clr, depth, 530, 530, 640/2-0.5, 480/2-0.5) ;

    Eigen::Vector3d n ;
    double d ;

    cv::Mat inliers ;

    det.findPlane(n, d, inliers) ;

    cv::imwrite("/tmp/mask0.png", inliers) ;

    vector<cv::Point> hull, hull2 ;
    cv::Mat dmap ;
    cv::Mat mask = det.findObjectMask(n, d, 0.01, dmap, hull) ;
    cv::Mat ref = det.refineSegmentation(clr, mask, hull2) ;

    dmap.convertTo(dmap, CV_8UC1);

    cv::Mat out ;
    cv::cornerEigenValsAndVecs(dmap, out, 9, 3) ;

    int w = dmap.cols, h = dmap.rows ;

    cv::Mat_<cv::Vec6f> out_(out) ;
    cv::Mat_<float> rmat(h, w), smat(h, w), prob(h, w) ;
    cv::Mat_<uchar> sign(h, w) ;


    for(int i=0 ; i<h ; i++ )
        for(int j=0 ; j<w ; j++ )
        {
            rmat[i][j] = 0 ;
            smat[i][j] = 0 ;
            prob[i][j] = 0 ;
            sign[i][j] = 0;

            float lambda_1_ = out_[i][j][0] ;
            float lambda_2_ = out_[i][j][1] ;



            float lambda_1 = std::min(fabs(lambda_1_), fabs(lambda_2_)) ;
            float lambda_2 = std::max(fabs(lambda_1_), fabs(lambda_2_)) ;

            if ( lambda_1_ * lambda_2_> 0 || lambda_2 < 1.0e-18 ) continue ;

            float r = fabs(lambda_1)/fabs(lambda_2) ;
            float s = sqrt(lambda_1_ * lambda_1_ + lambda_2_ * lambda_2_) ;

            rmat[i][j] = r ;
            smat[i][j] = s ;

            sign[i][j] = ( lambda_1_ < 0 ) ? 128 : 255 ;

            const double alpha = 0.5e-8, beta = 0.5 ;

            double p = exp(-r*r/2/beta/beta) * (1 - exp(-s*s/2/alpha/alpha)) ;

cout << s << endl ;

            prob[i][j] = p ;


        }

    cv::Mat rmat_ ;

    cv::convertScaleAbs(prob, rmat_, 1.0e8) ;

    cv::imwrite("/tmp/filtered.png", rmat_) ;
    cv::imwrite("/tmp/sign.png", sign) ;
}
