#include "ObjectOnPlaneDetector.h"

#include <highgui.h>
#include <iostream>



using namespace std;

int main(int argc, char *argv)
{
    loadSolver() ;
    cv::Mat clr = cv::imread("/home/malasiot/images/clothes/on_table/kinect_grab_000130_c.tif") ;
    cv::Mat depth = cv::imread("/home/malasiot/images/clothes/on_table/kinect_grab_000130_d.tif", -1) ;

    ObjectOnPlaneDetector det(clr, depth, 530, 530, 640/2-0.5, 480/2-0.5) ;

    Eigen::Vector3d n ;
    double d ;

    cv::Mat inliers ;

    det.findPlane(n, d, inliers) ;

    cv::imwrite("/tmp/mask0.png", inliers) ;

    vector<cv::Point> hull, hull2 ;
    cv::Mat mask = det.findObjectMask(n, d, 0.02, hull) ;
    cv::Mat ref = det.refineSegmentation(clr, mask, hull2) ;


    cv::imwrite("/tmp/refined.png", ref) ;




}
