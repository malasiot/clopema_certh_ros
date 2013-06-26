#include <ros/ros.h>

#include <camera_helpers/OpenNICapture.h>
#include <viz_helpers/CameraViewServer.h>
#include <robot_helpers/Utils.h>
#include <visualization_msgs/Marker.h>
#include <pcl/io/pcd_io.h>
#include <pcl/ros/conversions.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>

#include <fstream>

using namespace std ;

bool findLowestPoint2(const pcl::PointCloud<pcl::PointXYZ> &depth, const Eigen::Vector3d &orig, const Eigen::Vector3d &base, float apperture,
                     Eigen::Vector3d &p, Eigen::Vector3d &n) ;

void publishLowestPointMarker(ros::Publisher &vis_pub, const Eigen::Vector3d &p, const Eigen::Vector3d &n)
{


    visualization_msgs::Marker marker;
        // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = "/xtion3_rgb_optical_frame";
    marker.header.stamp = ros::Time::now();

    marker.ns = "lowest point";
    marker.id = 0;

    // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
    marker.type = visualization_msgs::Marker::ARROW ;

    Eigen::Vector3d ep = p + 0.3 *n ;

    geometry_msgs::Point p1, p2 ;
    p1.x = p.x() ;
    p1.y = p.y() ;
    p1.z = p.z() ;
    p2.x = ep.x() ;
    p2.y = ep.y() ;
    p2.z = ep.z() ;

    marker.points.push_back(p1) ;
    marker.points.push_back(p2) ;

    marker.scale.x = 0.01;
    marker.scale.y = 0.02;
    marker.scale.z = 0.0;

    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;

    marker.lifetime = ros::Duration();

    vis_pub.publish(marker);

}

int main(int argc, char **argv) {

    ros::init(argc, argv, "find_lowest");
    ros::NodeHandle nh;

    // query the transform between base and xtion3

    tf::TransformListener listener ;
    tf::StampedTransform transform ;
    Eigen::Affine3d t ;

    try {
        listener.waitForTransform("base_link", "xtion3_rgb_optical_frame", ros::Time(0), ros::Duration(10.0) );
        listener.lookupTransform("base_link", "xtion3_rgb_optical_frame",  ros::Time(0), transform);
        tf::TransformTFToEigen(transform, t);
    } catch (tf::TransformException ex) {
        ROS_ERROR("%s",ex.what());
    }

    // move the robot to the recorded target position

    Eigen::Vector3d orig(0.353086, -0.0402864, 1.10391) ;

    // coordinates in base frame ;
    Eigen::Vector3d orig_ = t * orig ;

    // compute the base of the search cone in xtion frame
    Eigen::Vector3d base = t.inverse() * ( orig_ + Eigen::Vector3d(0, 0, 0.7) ) ;

    ros::Publisher pub = nh.advertise<sensor_msgs::PointCloud2>( "rotating", 0 );

    ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 1);

    int count = 0 ;

    robot_helpers::MoveRobot rb ;
    robot_helpers::moveGripperPointingDown(rb, "r1", orig_.x(), orig_.y(), orig_.z()) ;

    // load point clouds and publish them

    while ( ros::ok() )
    {
        pcl::PointCloud<pcl::PointXYZ> cloud ;

        string fileName = "/home/malasiot/images/clothes/rotated/rot_right_one/cap_cloud_" + str(boost::format("%06d") % count) + ".pcd" ;

        if ( pcl::io::loadPCDFile(fileName, cloud) < 0 ) {
            count = 0 ;
            continue ;
        }

        Eigen::Vector3d p, n ;

        if ( findLowestPoint2(cloud, orig, base, M_PI/4, p, n) )
            publishLowestPointMarker(marker_pub, p, n) ;

        sensor_msgs::PointCloud2 msg ;

        pcl::toROSMsg(cloud, msg) ;

        msg.header.frame_id = "xtion3_rgb_optical_frame" ;

        pub.publish(msg) ;

        ros::spinOnce() ;

        count ++ ;

    }

}
