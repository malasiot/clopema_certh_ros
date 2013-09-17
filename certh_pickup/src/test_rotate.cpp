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


class RotateAndGrab {
public:
    RotateAndGrab(const string &camera_, const string &arm_): cap(camera_), camera(camera_), arm(arm_), captureStoped(false), counter(0) {

    }

    void start()
    {

        cap.connect() ;
        cmove.actionStarted.connect(boost::bind(&RotateAndGrab::startCapture, this)) ;
        cmove.actionCompleted.connect(boost::bind(&RotateAndGrab::stopCapture, this)) ;

        rotateGripper(cmove, arm, 2*M_PI) ;

        while (!captureStoped ) ;
    }

    void startCapture()
    {
        captureStoped = false ;
        boost::thread capture_thread(boost::bind(&RotateAndGrab::doCapture, this)) ;

    }


    void doCapture()
    {
        bool _stoped = false ;

        tf::TransformListener listener(ros::Duration(1.0));

        do {

            {
                boost::unique_lock<boost::mutex> lock_ ;
                _stoped = captureStoped ;
            }

            if ( !_stoped )
            {

                cv::Mat clr, depth ;
                pcl::PointCloud<pcl::PointXYZ> pc ;
                ros::Time ts ;
                image_geometry::PinholeCameraModel cm;
    /*
                string rgbFileName = "/tmp/rot/" + str(boost::format("cap_rgb_%06d.png") % counter) ;
                string depthFileName = "/tmp/rot/" + str(boost::format("cap_depth_%06d.png") % counter) ;
                string cloudFileName = "/tmp/rot/" + str(boost::format("cap_cloud_%06d.pcd") % counter) ;
                */

                if ( cap.grab(clr, depth, pc, ts, cm) )
                {

                    cout << counter << endl ;

                    tf::StampedTransform transform;

                    Eigen::Affine3d pose ;

                    try {
                        listener.waitForTransform("xtion3_rgb_optical_frame","r1_ee", ts, ros::Duration(1) );
                        listener.lookupTransform("xtion3_rgb_optical_frame","r1_ee", ts, transform);

    //                    listener.waitForTransform("r1_ee", "base_link", ts, ros::Duration(1) );
    //                    listener.lookupTransform("r1_ee", "base_link", ts, transform);

               //         cv::imwrite(rgbFileName, clr) ;
               //         cv::imwrite(depthFileName, depth) ;
               //         pcl::io::savePCDFileBinary(cloudFileName, pc) ;
                        tf::TransformTFToEigen(transform, pose);

                 //       cap_data << counter << endl << pose.matrix() << endl ;

                        counter ++ ;

                    } catch (tf::TransformException ex) {
                        ROS_INFO("%s",ex.what());
                    }


                }

            }

        } while ( !_stoped ) ;

    }

    // set the robot speed to 1.3 for ~200 pics
    void stopCapture()
    {
        cout << "done" << endl ;

        boost::unique_lock<boost::mutex> lock_ ;
        captureStoped = true ;
        finished.notify_all();

        robot_helpers::setServoPowerOff() ;

    }



    MoveRobot cmove ;
    camera_helpers::OpenNICaptureAll cap ;
    string camera, arm ;
    bool captureStoped ;

    boost::mutex captureMutex ;
    boost::condition_variable finished ;
    int counter ;


};








int main(int argc, char **argv) {
    ros::init(argc, argv, "move_rotate_grab");
    ros::NodeHandle nh;

    MoveRobot cmove ;
    cmove.setServoMode(false);

  // moveHome() ;
    moveGripperPointingDown(cmove, "r1", 0, -0.7, 1.5) ;

    RotateAndGrab rg("xtion3", "r1") ;

    rg.start() ;


    ros::spin() ;





}
