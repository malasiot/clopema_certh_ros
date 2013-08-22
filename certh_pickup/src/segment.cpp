#include "ObjectOnPlaneDetector.h"

#include <highgui.h>
#include <iostream>

#include <certh_libs/Point2D.h>

using namespace std ;
using namespace certh_libs ;

void findInitialCandidates(const cv::Mat &cim_, const cv::Mat &dim_, vector<Point2D> &candidates) ;

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

    vector<Point2D> pts ;
    findInitialCandidates(clr, dmap, pts);

}
