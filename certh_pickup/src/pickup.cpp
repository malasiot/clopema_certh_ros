#include <ros/ros.h>

#include <robot_helpers/Robot.h>
#include <robot_helpers/Utils.h>

#include <camera_helpers/OpenNICapture.h>
#include <pcl/io/pcd_io.h>
#include <highgui.h>

#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>

#include "ObjectOnPlaneDetector.h"
#include "RidgeDetector.h"

#include <highgui.h>
#include <iostream>

#include <certh_libs/Point2D.h>

using namespace std;
using namespace robot_helpers ;
using namespace Eigen ;

const double defaultDist = 0.9 ;
const double defaultTableHeight = 0.75 ;
const double gripperOffset = 0.2 ;


Affine3d getSensorPose(const tf::TransformListener &listener, const string &camera)
{
    tf::StampedTransform transform;
    Affine3d pose ;

    try {
        listener.waitForTransform(camera + "_rgb_optical_frame", "base_link", ros::Time(0), ros::Duration(1) );
        listener.lookupTransform(camera + "_rgb_optical_frame", "base_link", ros::Time(0), transform);

        tf::TransformTFToEigen(transform, pose);

        return pose ;
    } catch (tf::TransformException ex) {
        ROS_ERROR("%s",ex.what());
        return pose ;
    }
}

double getTableHeightFromPlane(const Vector3d &n, double d, const Affine3d &frame)
{
    return n.dot(frame.translation()) + d ;
}

Vector3d retargetSensor(const cv::Mat dmap, const vector<cv::Point> &hull, const image_geometry::PinholeCameraModel &cm, double Zd)
{
    double minZ, maxZ, Z ;

    cv::minMaxLoc(dmap, &minZ, &maxZ) ;

    // compute the minimum distance of the sensor from the table

    minZ = maxZ/1000.0 + 0.7 ;

    // find the center of the object and re-target the sensor

    cv::Point2f center ;
    float rad ;
    cv::minEnclosingCircle(hull, center, rad) ;

    double zS = Zd * rad / std::min(dmap.cols, dmap.rows) ;

    zS = std::max(minZ, zS) ;

    cv::Point3d p = cm.projectPixelTo3dRay(cv::Point2d(center.x, center.y));

    p.x *= zS ; p.y *= zS ; p.z *= zS ;

    return Vector3d(p.x, p.y, p.z) ;

}

const string armName = "r2" ;

int main(int argc, char **argv) {
    ros::init(argc, argv, "pickup");
    ros::NodeHandle nh;

    MoveRobot cmove ;
    moveGripperPointingDown(cmove, armName, 0, -0.9, defaultDist + defaultTableHeight - gripperOffset );

    string camera = ( armName == "r1" ) ? "xtion1" : "xtion2" ;
    camera_helpers::OpenNICaptureRGBD grabber(camera) ;

    // open the camera

    grabber.connect() ;

    ros::Time ts ;
    image_geometry::PinholeCameraModel cm ;

    // grab frame

    cv::Mat clr, depth ;
    grabber.grab(clr, depth, ts, cm) ;

    clr = cv::imread("/home/malasiot/images/clothes/calibration/on_table/cap_3/cap_rgb_000009.png") ;
    depth = cv::imread("/home/malasiot/images/clothes/calibration/on_table/cap_3/cap_depth_000009.png", -1) ;

    // detect table and object

    ObjectOnPlaneDetector objDet(depth, cm.fx(), cm.fy(), cm.cx(), cm.cy()) ;

    Eigen::Vector3d n ;
    double d ;
    cv::Mat inliers ;

    if ( !objDet.findPlane(n, d, inliers) ) return 0 ;

    vector<cv::Point> hull ;
    cv::Mat dmap ;
    cv::Mat mask = objDet.findObjectMask(n, d, 0.01, dmap, hull) ;

    tf::TransformListener listener ;
    Affine3d frame = getSensorPose(listener, camera) ;
    double tableHeight = getTableHeightFromPlane(n, d, frame) ;

    Vector3d t = retargetSensor(dmap, hull, cm, defaultDist - defaultTableHeight) ;

    double newDist = t.z() + defaultTableHeight - gripperOffset ;

    if ( newDist < defaultTableHeight ) return 0 ;

    moveGripperPointingDown(cmove, armName, t.x(), -0.9 + t.y(), newDist );

    grabber.grab(clr, depth, ts, cm) ;

    cv::imwrite("/tmp/grabber_c.png", clr) ;
    cv::imwrite("/tmp/grabber_d.png", depth) ;

    grabber.disconnect();
}
