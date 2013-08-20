#include "calibration.h"

#include <ros/ros.h>

#include <camera_helpers/OpenNICapture.h>
#include <robot_helpers/Utils.h>
#include <robot_helpers/Geometry.h>
#include <certh_libs/Helpers.h>

#include <boost/filesystem.hpp>
#include <fstream>

#include <highgui.h>

using namespace std ;
using namespace Eigen ;

string armName = "r2", camera_id, outFolder, dataFolder = "/tmp/";
int nStations = 20 ;
cv::Size boardSize(3, 5) ;
double cellSize = 0.04 ;

double minX = -0.25 ;
double maxX = 0.25 ;
double minY = -1.3 ;
double maxY = -0.8 ;
double minZ = 0.9 ;
double maxZ = 1.5 ;

Vector3d orient(-1, 0, 0) ;


int main(int argc, char **argv) {

    ros::init(argc, argv, "handeye_calibration");
    ros::NodeHandle nh ;

    srand(clock()) ;

    int c = 1 ;

    while ( c < argc )
    {
        if ( strncmp(argv[c], "--camera", 8) == 0 )
        {
            if ( c + 1 < argc ) {
                camera_id = argv[++c] ;
            }
            ++c ;
        }
        else if ( strncmp(argv[c], "--out", 5) == 0 )
        {
            if ( c + 1 < argc ) {
                outFolder = argv[++c] ;
            }
            ++c ;
        }
        else if ( strncmp(argv[c], "--data", 6) == 0 )
        {
            if ( c + 1 < argc ) {
                dataFolder = argv[++c] ;
            }
            ++c ;
        }
        else if ( strncmp(argv[c], "--board", 7) == 0 )
        {
            int bx = -1, by = -1 ;

            if ( c + 1 < argc ) {
                bx = atof(argv[++c]) ;
            }
            if ( c + 1 < argc ) {
                by = atof(argv[++c]) ;
            }

            if ( bx > 0 && by > 0 )
                boardSize = cv::Size(bx, by) ;
            ++c ;
        }
        else if ( strncmp(argv[c], "--cell", 7) == 0 )
        {
            if ( c + 1 < argc ) {
                string cs = argv[++c] ;
                cellSize = atof(cs.c_str()) ;;
            }
            ++c ;
        }
        else if ( strncmp(argv[c], "--stations", 10) == 0 )
        {
            if ( c + 1 < argc ) {
                nStations = atoi(argv[++c]) ;
            }
            ++c ;
        }
        else if ( strncmp(argv[c], "--bbox", 6) == 0 )
        {
            if ( c + 1 < argc ) {
                minX = atof(argv[++c]) ;
            }
            if ( c + 1 < argc ) {
                minY = atof(argv[++c]) ;
            }
            if ( c + 1 < argc ) {
                minZ = atof(argv[++c]) ;
            }
            if ( c + 1 < argc ) {
                maxX = atof(argv[++c]) ;
            }
            if ( c + 1 < argc ) {
                maxY = atof(argv[++c]) ;
            }
            if ( c + 1 < argc ) {
                maxZ = atof(argv[++c]) ;
            }

            ++c ;
        }
        else if ( strncmp(argv[c], "--orient", 8) == 0 )
        {
            double ox, oy, oz ;

            if ( c + 1 < argc ) {
                ox = atof(argv[++c]) ;
            }
            if ( c + 1 < argc ) {
                oy = atof(argv[++c]) ;
            }
            if ( c + 1 < argc ) {
                oz = atof(argv[++c]) ;
            }

            orient = Vector3d(ox, oy, oz) ;

            ++c ;
        }
    }


    if ( camera_id.empty() )
    {
        ROS_ERROR("No camera specified") ;
        return 0 ;
    }

    if ( outFolder.empty() )
    {
        outFolder = "/home/" ;
        outFolder += getenv("USER") ;
        outFolder += "/.ros/clopema_calibration/" ;
        outFolder += camera_id ;
        outFolder += "/handeye_rgb.calib" ;
    }

    ros::AsyncSpinner spinner(4) ;
    spinner.start() ;

    // open grabber and wait until connection

    camera_helpers::OpenNICaptureRGBD grabber(camera_id) ;

    if ( !grabber.connect(ros::Duration(5.0)) )
    {
        ROS_ERROR("Cannot connect to frame grabber: %s", camera_id.c_str()) ;
        return 0 ;
    }

    c = 0 ;

    // move robot and grab images

    robot_helpers::MoveRobot mv ;
    mv.setServoMode(false);

    double cx, cy, fx, fy ;

    while ( c < nStations )
    {
        // move the robot to the next position

        double X = minX + (maxX - minX)*(rand()/(double)RAND_MAX) ;
        double Y = minY + (maxY - minY)*(rand()/(double)RAND_MAX) ;
        double Z = minZ + (maxZ - minZ)*(rand()/(double)RAND_MAX) ;

        const double qscale = 0.3 ;
        double qx = qscale * (rand()/(double)RAND_MAX - 0.5) ;
        double qy = qscale * (rand()/(double)RAND_MAX - 0.5) ;
        double qz = qscale * (rand()/(double)RAND_MAX - 0.5) ;
        double qw = qscale * (rand()/(double)RAND_MAX - 0.5) ;

        Quaterniond q = robot_helpers::lookAt(orient, M_PI) ;

        q = Quaterniond(q.x() + qx, q.y() + qy, q.z() + qz, q.w() + qw) ;
        q.normalize();

        if ( robot_helpers::moveGripper(mv, armName, Eigen::Vector3d(X, Y, Z), q) )
        {
            image_geometry::PinholeCameraModel cm ;
            cv::Mat clr, depth ;

            ros::Time ts ;

            grabber.grab(clr, depth, ts, cm) ;

            // camera intrinsics. we assume a rectfied and calibrated frame

            cx = cm.cx() ;
            cy = cm.cy() ;
            fx = cm.fx() ;
            fy = cm.fy() ;

            string filenamePrefix = dataFolder + str(boost::format("/grab_%06d") % c) ;

            cv::imwrite(filenamePrefix + "_c.png", clr) ;
            cv::imwrite(filenamePrefix + "_d.png", depth) ;

            Eigen::Affine3d pose_ = robot_helpers::getPose(armName) ;

            {
                ofstream strm((filenamePrefix + "_pose.txt").c_str()) ;

                strm << pose_.rotation() << endl << pose_.translation() ;
            }

            ++c ;

        }
        else continue ;
    }

    vector<Affine3d> gripper_to_base, target_to_sensor ;
    Affine3d sensor_to_base ;

    cv::Mat cameraMatrix ;

    cameraMatrix = cv::Mat::eye(3, 3, CV_64F);

    cameraMatrix.at<double>(0, 0) = fx ;
    cameraMatrix.at<double>(1, 1) = fy ;
    cameraMatrix.at<double>(0, 2) = cx ;
    cameraMatrix.at<double>(1, 2) = cy ;

    find_target_motions("grab_",  "/home/malasiot/images/clothes/calibration/", boardSize, cellSize, cameraMatrix, true, gripper_to_base, target_to_sensor) ;

  //  find_target_motions("grab_",  dataFolder, boardSize, cellSize, true, gripper_to_base, target_to_sensor) ;

    solveHandEye(gripper_to_base, target_to_sensor, Tsai, true, sensor_to_base) ;

    certh_libs::createDir(boost::filesystem::path(outFolder).parent_path().string(), true) ;

    ofstream ostrm(outFolder.c_str()) ;

    ostrm << sensor_to_base.inverse().matrix() ;

    return 1;
}
