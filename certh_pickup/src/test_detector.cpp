#include "ObjectOnPlaneDetector.h"
#include "RidgeDetector.h"

#include <highgui.h>
#include <iostream>

#include <certh_libs/Point2D.h>

using namespace std;
using namespace cv ;
using namespace Eigen ;


int main(int argc, char **argv)
{
    Mat clr, depth ;

    clr = imread("/home/malasiot/images/clothes/calibration/on_table/cap_3/cap_rgb_000009.png") ;
    depth = imread("/home/malasiot/images/clothes/calibration/on_table/cap_3/cap_depth_000009.png", -1) ;

    ObjectOnPlaneDetector objDet(depth, 525, 525, 640/2.0, 480/2) ;

    Eigen::Vector3d n ;
    double d ;

    if ( !objDet.findPlane(n, d) ) return 0 ;

    vector<cv::Point> hull ;
    cv::Mat dmap ;
    cv::Mat mask = objDet.findObjectMask(n, d, 0.01, dmap, hull) ;

    RidgeDetector rdg ;
    vector<RidgeDetector::GraspCandidate> gsp ;

    rdg.detect(dmap, gsp) ;
    rdg.draw(clr, gsp) ;

    cv::imwrite("/tmp/gsp.png", clr) ;
}
