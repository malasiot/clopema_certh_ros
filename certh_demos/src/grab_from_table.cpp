#include <ros/ros.h>

#include <robot_helpers/Robot.h>
#include <robot_helpers/Utils.h>

#include <camera_helpers/OpenNICapture.h>
#include <pcl/io/pcd_io.h>
#include <highgui.h>

#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>

using namespace std;
using namespace robot_helpers ;

bool captureStoped ;
boost::mutex captureMutex ;
boost::condition_variable finished ;
int counter = 0;

bool grabpc(camera_helpers::OpenNICaptureAll *grabber)
{
    while(counter < 10){
        cv::Mat rgb, depth ;
        pcl::PointCloud<pcl::PointXYZ> pc ;
        ros::Time ts ;
        image_geometry::PinholeCameraModel cm ;

        string rgbFileName = "/tmp/table/" + str(boost::format("cap_rgb_%06d.png") % counter) ;
        string depthFileName = "/tmp/table/" + str(boost::format("cap_depth_%06d.png") % counter) ;
        string cloudFileName = "/tmp/table/" + str(boost::format("cap_cloud_%06d.pcd") % counter) ;
        if ( grabber->grab(rgb, depth, pc, ts, cm) )
        {
            cv::imwrite(rgbFileName, rgb) ;
            cv::imwrite(depthFileName, depth) ;
            pcl::io::savePCDFileBinary(cloudFileName, pc) ;
        }
        counter++;
        cout<< "hit enter to continue";
        cin.ignore();
    }
    grabber->disconnect() ;

    return true;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "grab_from_table");
    ros::NodeHandle nh;

    MoveRobot cmove ;
    moveGripperPointingDown(cmove, "r2", 0, -0.9, 1.4 );
    camera_helpers::OpenNICaptureAll grabber2("xtion2") ;
    grabber2.connect() ;
    grabpc(&grabber2) ;


    return 1;
}


