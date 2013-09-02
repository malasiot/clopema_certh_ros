#include "ObjectOnPlaneDetector.h"
#include "RidgeDetector.h"
#include <highgui.h>
#include <iostream>
#include <robot_helpers/Robot.h>
#include <robot_helpers/Utils.h>
#include <certh_libs/Point2D.h>
#include <camera_helpers/OpenNICapture.h>

using namespace std ;
using namespace certh_libs ;
using namespace robot_helpers ;
using namespace std;

Eigen::Vector3d findTarget(float x , float y, pcl::PointCloud<pcl::PointXYZ> pc ){

    Eigen::Vector3d v;



}

int main(int argc, char **argv) {


    ros::init(argc, argv, "unfolding");
    ros::NodeHandle nh;
//    cv::Mat clr = cv::imread("/home/malasiot/images/clothes/calibration/on_table/cap_3/cap_rgb_000009.png") ;
//    cv::Mat depth = cv::imread("/home/malasiot/images/clothes/calibration/on_table/cap_3/cap_depth_000009.png", -1) ;
//
    string armName = "r2";
    MoveRobot cmove;
    cmove.setServoMode(false);
    setGripperState(armName, true);
    moveGripperPointingDown(cmove, armName, 0, -1.1, 1.3 );

    camera_helpers::OpenNICaptureAll grabber("xtion2");
    grabber.connect();
    ros::Duration(0.3).sleep();

    cv::Mat rgb, depth;
    pcl::PointCloud<pcl::PointXYZ> pc;
    ros::Time ts ;
    image_geometry::PinholeCameraModel cm;

    if(!grabber.grab(rgb, depth, pc, ts, cm)){
        cout<<" Cant grab image!!  " << endl;
        return false;
    }

  //  grabber.disconnect();
//
    ObjectOnPlaneDetector det(depth, 525, 525, 640/2-0.5, 480/2-0.5) ;

    Eigen::Vector3d n ;
    double d ;

    cv::Mat inliers ;

    det.findPlane(n, d, inliers) ;

    vector<cv::Point> hull, hull2 ;
    cv::Mat dmap ;
    cv::Mat mask = det.findObjectMask(n, d, 0.01, dmap, hull) ;
    cv::Mat ref = det.refineSegmentation(rgb, mask, hull2) ;

    RidgeDetector rdg ;
    vector<RidgeDetector::GraspCandidate> gsp ;

    rdg.detect(dmap, gsp) ;
    rdg.draw(rgb, gsp) ;


    cv::imwrite("/tmp/gsp.png", rgb) ;
   // cout << "point is " <<gsp[0].x << " "<<  gsp[0].y << endl;

    ////////////////
    pcl::PointXYZ val = pc.at(gsp[0].y, gsp[0].x) ;

    Eigen::Vector3d p(val.x, val.y, val.z) ;
   // cout << p << endl;
    Eigen::Matrix4d calib = getTranformationMatrix("xtion2_rgb_optical_frame");
    Eigen::Vector4d tar(p.x(), p.y(), p.z(), 1);
    Eigen::Vector4d targetP;

    targetP = calib * tar;
    cout << targetP << endl;

    moveGripperPointingDown(cmove, armName, targetP.x(),targetP.y(),targetP.z()+0.1);
    setServoPowerOff();
    /////////////////
    return 0;

}
