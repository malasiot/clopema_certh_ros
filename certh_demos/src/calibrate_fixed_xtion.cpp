#include <ros/ros.h>

#include <camera_helpers/OpenNICapture.h>
#include <viz_helpers/CameraViewServer.h>
#include <robot_helpers/Utils.h>
#include <robot_helpers/Geometry.h>
#include <visualization_msgs/Marker.h>

#include <fstream>
#include <highgui.h>

using namespace std ;
using namespace Eigen ;

bool calibration_finished ;

float points[9][3] = {
    { -0.25, 	-0.9,   1.5 },
    { -0.15,   -0.9,   1.5 },
    { 0.2,  	-0.9,   1.5},
    {-0.5, 	-1.1,	1.2 },
    {-0.15,   -1.1, 	1.2},
    {0.2,  	-1.1, 	1.2},
    {-0.5, 	-1.3, 	0.9},
    {-0.15,   -1.3, 	0.9},
    {0.2,  	-1.3, 	0.9}
} ;

const double minX = -0.25 ;
const double maxX = 0.25 ;
const double minY = -1.3 ;
const double maxY = -0.8 ;
const double minZ = 0.9 ;
const double maxZ = 1.5 ;

const int maxPoints = 20 ;

void doCalibrate(const vector<pcl::PointXYZ> &pts)
{
    cv::Mat meanRobot = cv::Mat(3, 1, CV_32FC1, cv::Scalar::all(0));

    int n = pts.size() ;

    for (unsigned int i=0;i<n;i++){
        for (unsigned int j=0;j<3;j++){
            meanRobot.at<float>(j, 0) += (points[i][j]/(float)n);
        }
    }

    cv::Mat xtionPoints(n, 3, CV_32FC1, cv::Scalar::all(0));

    cv::Mat meanXtion = cv::Mat(3, 1, CV_32FC1, cv::Scalar::all(0));

    for (unsigned int i=0;i<n;i++)
    {
        const pcl::PointXYZ &p = pts[i] ;

        xtionPoints.at<float>(i, 0) = p.x ;
        xtionPoints.at<float>(i, 1) = p.y ;
        xtionPoints.at<float>(i, 2) = p.z ;

        //cout << "Xtion points: " << p.x << " " << p.y << " " << p.z << std::endl;

        meanXtion.at<float>(0, 0) += p.x/n;
        meanXtion.at<float>(1, 0) += p.y/n;
        meanXtion.at<float>(2, 0) += p.z/n;
    }


    cv::Mat H = cv::Mat(3, 3, CV_32FC1, cv::Scalar::all(0));
    cv::Mat Pa = cv::Mat(3, 1, CV_32FC1, cv::Scalar::all(0));
    cv::Mat Pb = cv::Mat(1, 3, CV_32FC1, cv::Scalar::all(0));

    for(int i=0; i<n; ++i){
        for(int j=0; j<3; ++j){
            Pa.at<float>(j, 0) = xtionPoints.at<float>(i, j) - meanXtion.at<float>(j, 0);
            Pb.at<float>(0, j) = points[i][j] - meanRobot.at<float>(j, 0);
        }
        H += Pa * Pb;
    }
    cv::SVD svd(H, cv::SVD::FULL_UV);

    cv::Mat tr(4, 4, CV_32FC1, cv::Scalar::all(0)) ;
    cv::Mat R = svd.vt.t() * svd.u.t();
    cv::Mat t = (-1)*R*meanXtion + meanRobot;

    for(int i=0; i<3; ++i)
        for(int j=0; j<3; ++j)
            tr.at<float>(i, j) = R.at<float>(i, j) ;

    for(int i=0 ; i<3 ; i++)
        tr.at<float>(i, 3) = t.at<float>(i, 0) ;

    tr.at<float>(3, 3) = 1.0 ;

    cv::Mat trInv = tr.inv() ;

    string outFolder = getenv("HOME") ;
    outFolder += "/.ros/clopema_calibration/handeye/xtion3/calib.txt" ;

    ofstream fout(outFolder.c_str());

    for(int i=0; i<4; ++i) {
        for(int j=0; j<4; ++j)
            fout << trInv.at<float>(i, j) << " " ;

        fout << endl;
    }
}

struct Context {
    int currentPoint ;
    camera_helpers::OpenNICaptureAll *grabber ;
    vector<pcl::PointXYZ> pts ;
};


int counter = 0 ;

void onMouseClicked(int x, int y, Context *ctx)
{
    //cout << x << ' ' << y << endl ;

    // capture point cloud

    pcl::PointCloud<pcl::PointXYZ> cloud ;
    image_geometry::PinholeCameraModel cm ;
    cv::Mat clr, depth ;

    ros::Time ts ;

    ctx->grabber->grab(clr, depth, cloud, ts, cm) ;

    string filenamePrefix = str(boost::format("/tmp/grab_%06d") % counter++) ;

    cv::imwrite(filenamePrefix + "_c.png", clr) ;
    cv::imwrite(filenamePrefix + "_d.png", depth) ;

    pcl::io::savePCDFileBinary(filenamePrefix + ".pcd", cloud) ;

    Eigen::Affine3d pose_ = robot_helpers::getCurrentPose("r1") ;

    {
        ofstream strm((filenamePrefix + "_pose.txt").c_str()) ;

        strm << pose_.rotation() << endl << pose_.translation() ;
    }

    // get the 3D coordinate on clicked position

    pcl::PointXYZ p = cloud.at(x, y);

    //cout << p << endl ;

    // add the point to the list

    ctx->pts.push_back(p) ;

    ctx->currentPoint++ ;

    if ( ctx->currentPoint == maxPoints )
    {
        // all points have been added, do the calibration

        doCalibrate(ctx->pts) ;
        calibration_finished = true ;
        return ;
    }
    else
    {
        // move the robot to the next position

        int c = ctx->currentPoint ;

        robot_helpers::MoveRobot mv ;
        mv.setServoMode(false);

        while (1)
        {
            double X = minX + (maxX - minX)*(rand()/(double)RAND_MAX) ;
            double Y = minY + (maxY - minY)*(rand()/(double)RAND_MAX) ;
            double Z = minZ + (maxZ - minZ)*(rand()/(double)RAND_MAX) ;

            const double qscale = 0.3 ;
            double qx = qscale * (rand()/(double)RAND_MAX - 0.5) ;
            double qy = qscale * (rand()/(double)RAND_MAX - 0.5) ;
            double qz = qscale * (rand()/(double)RAND_MAX - 0.5) ;
            double qw = qscale * (rand()/(double)RAND_MAX - 0.5) ;

            Quaterniond q = robot_helpers::lookAt(Vector3d(-1, 0, 0), M_PI) ;

            q = Quaterniond(q.x() + qx, q.y() + qy, q.z() + qz, q.w() + qw) ;
            q.normalize();

            if ( robot_helpers::moveGripper(mv, "r2", Eigen::Vector3d(X, Y, Z), q) ) break ;

        }
    }

}


int main(int argc, char **argv) {

    ros::init(argc, argv, "calibrate_fixed_xtion");
    ros::NodeHandle nh;

    srand(clock()) ;

    ros::Publisher vis_pub = nh.advertise<visualization_msgs::Marker>( "visualization_marker", 0 );

    viz_helpers::CameraViewServer srv ;

    // open grabber and wait until connection
    camera_helpers::OpenNICaptureAll grabber("xtion3") ;
    grabber.connect() ;

    Context ctx ;
    ctx.currentPoint = 0 ;
    ctx.grabber = &grabber ;

    Eigen::Quaterniond q ;
     q = Eigen::AngleAxisd(-M_PI/2, Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(-M_PI/2, Eigen::Vector3d::UnitZ());

     // move the robot to the first position

    robot_helpers::MoveRobot mv ;
    robot_helpers::moveGripper(mv, "r2", Eigen::Vector3d(points[0][0], points[0][1], points[0][2]), q) ;

    // enable manual tip detection

    srv.mouseClicked.connect(boost::bind(onMouseClicked, _1, _2, &ctx));

    // wait until calibration has finished

    while (!calibration_finished) ;


    return 1;
}
