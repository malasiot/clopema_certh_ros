#include "ObjectOnPlaneDetector.h"
#include "RidgeDetector.h"

#include <highgui.h>
#include <iostream>

#include <certh_libs/Point2D.h>

using namespace std ;
using namespace certh_libs ;

using namespace std;

int main(int argc, char *argv)
{

    cv::Mat clr = cv::imread("/home/malasiot/images/clothes/calibration/on_table/cap_3/cap_rgb_000009.png") ;
    cv::Mat depth = cv::imread("/home/malasiot/images/clothes/calibration/on_table/cap_3/cap_depth_000009.png", -1) ;

    ObjectOnPlaneDetector det(depth, 525, 525, 640/2-0.5, 480/2-0.5) ;

    Eigen::Vector3d n ;
    double d ;

    cv::Mat inliers ;

    det.findPlane(n, d, inliers) ;

    vector<cv::Point> hull, hull2 ;
    cv::Mat dmap ;
    cv::Mat mask = det.findObjectMask(n, d, 0.01, dmap, hull) ;
    cv::Mat ref = det.refineSegmentation(clr, mask, hull2) ;

    RidgeDetector rdg ;
    vector<RidgeDetector::GraspCandidate> gsp ;

    rdg.detect(dmap, gsp) ;
    rdg.draw(clr, gsp) ;

    cv::imwrite("/tmp/gsp.png", clr) ;

}
