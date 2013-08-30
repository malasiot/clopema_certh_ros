#include "Unfold.h"
#include <opencv2/highgui/highgui.hpp>

#include <cv.h>

#include <Eigen/Core>
#include <Eigen/Eigenvalues> // for cwise access


#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
using namespace robot_helpers;
using namespace std;


Eigen::Vector3d findHighestPoint( pcl::PointCloud<pcl::PointXYZ> depth ){

    int w = depth.width, h = depth.height;

    int best_j = -1, best_i = -1 ;

    Eigen::Vector3d best;
    best << 0,0, 100000000;

    bool found = false;

    for(int j=0 ; j<w ; j++ )
    {

        for(int i=0 ; i<h ; i++)
        {
            pcl::PointXYZ val = depth.at(j, i);

            if ( !pcl_isfinite(val.z) ) continue;
            if (val.z < 0.4) continue;

            Eigen::Vector3d p(val.x, val.y, val.z);

            if ( val.z < best.z() )
            {
                best << val.x, val.y, val.z;
                found = true;
            }

        }
    }

    return best;

}


int main(int argc, char **argv) {

    ros::init(argc, argv, "grab_from_table");
    ros::NodeHandle nh;

    MoveRobot cmove;
    cmove.setServoMode(false);
      setGripperState("r2", true);
    moveGripperPointingDown(cmove, "r2", 0, -1.1, 1.1 );

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

    Eigen::Vector3d p;
    p = findHighestPoint(pc );
     Eigen::Vector4d targetP;
    Eigen::Matrix4d calib = getTranformationMatrix("xtion2_rgb_optical_frame");

    Eigen::Vector4d tar(p.x(), p.y(), p.z(), 1);
    targetP = calib * tar;

    cout << targetP<< endl;


    moveGripperPointingDown(cmove, "r2", targetP.x(), targetP.y(), targetP.z()+0.1);

    moveGripperPointingDown(cmove, "r2", targetP.x(), targetP.y(), targetP.z()-0.05);
    setGripperState("r2", false);
    moveHomeArm("r2");
    grabber.disconnect();
    //Set servo power off
        clopema_motoros::SetPowerOff soff;
        soff.request.force = false;
        ros::service::waitForService("/joint_trajectory_action/set_power_off");
        if (!ros::service::call("/joint_trajectory_action/set_power_off", soff)) {
            ROS_ERROR("Can't call service set_power_off");
            return -1;
        }
    return 0;
}
