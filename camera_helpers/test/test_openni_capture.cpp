#include <camera_helpers/OpenNICapture.h>
#include <camera_helpers/OpenNIServiceClient.h>

#include <string>
#include <highgui.h>
#include <pcl/io/pcd_io.h>
#include <boost/thread.hpp>

using namespace Eigen ;
using namespace std ;


class RotateAndGrab {
public:

    RotateAndGrab(const std::string &camera_, const std::string &arm_) ;

    void init() ;

    void rotate(double theta) ;

protected:

    virtual void process(const cv::Mat &clr, const cv::Mat &depth, const image_geometry::PinholeCameraModel &cm,
                         const ros::Time &ts, Eigen::Affine3d &tip_pose_in_camera_frame) {}


    void startCapture() ;
    void doCapture() ;
    void stopCapture() ;

    camera_helpers::OpenNICaptureRGBD cap ;

    std::string camera, arm ;
    bool captureStoped ;
    boost::mutex mutex ;
    boost::thread capture_thread ;
    int counter ;
};


RotateAndGrab::RotateAndGrab(const string &camera_, const string &arm_): cap(camera_), camera(camera_), arm(arm_) {

}

void RotateAndGrab::init()
{
    //moveGripper(cmove, arm, pos, lookAt(Vector3d(0, 0, -1), 0)) ;

    camera_helpers::openni::connect(camera) ;

    counter = 0 ;
    startCapture() ;
    //cmove.actionStarted.connect(boost::bind(&RotateAndGrab::startCapture, this)) ;
    //cmove.actionCompleted.connect(boost::bind(&RotateAndGrab::stopCapture, this)) ;
}

void RotateAndGrab::rotate(double theta)
{

    while (!captureStoped ) ;

    capture_thread.join() ;
}

void RotateAndGrab::startCapture()
{
    captureStoped = false ;
    capture_thread = boost::thread(boost::bind(&RotateAndGrab::doCapture, this)) ;
}

void RotateAndGrab::doCapture()
{
    bool _stoped = false ;


    do {

        {
            boost::mutex::scoped_lock lock_(mutex) ;
            _stoped = captureStoped ;
        }

        if ( !_stoped )
        {
            cv::Mat clr, depth ;
            ros::Time ts ;
            image_geometry::PinholeCameraModel cm;

            if (  camera_helpers::openni::grab(camera, clr, depth, ts, cm) )
            {

                cout << counter++ << endl ;

            }

        }

    } while ( !_stoped ) ;

}

void RotateAndGrab::stopCapture()
{
    boost::mutex::scoped_lock lock_(mutex) ;
    captureStoped = true ;
}


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "openni_capture_example");
    ros::NodeHandle nh ;


 //   camera_helpers::OpenNICapturePointCloud grabber2("xtion3") ;

    //grabber2.connect(boost::bind( &grabpc, &grabber2 )) ;

    RotateAndGrab rg("xtion2", "r2") ;
    rg.init() ;

    rg.rotate(30) ;

    camera_helpers::OpenNICaptureRGBD grabber("xtion2") ;

    image_geometry::PinholeCameraModel camera ;

    if ( grabber.connect( ) )
    {
        std::cout << "start grabbing RGBD" << std::endl ;

        cv::Mat clr, depth ;
        ros::Time ts ;

        for(int i=0 ; i<10 ; i++ )
        {
            if ( grabber.grab(clr, depth, ts, camera) )
            {

                cv::imwrite(str(boost::format("/tmp/rgb_%03d.png") % i), clr) ;
                cv::imwrite(str(boost::format("/tmp/depth_%03d.png") % i), depth) ;
                std::cout << "grab" << std::endl ;
            }
            else { std::cout << "ok" << std::endl ; }

            ros::Duration(1).sleep() ;
        }

        std::cout << "finished RGBD" << std::endl ;

        grabber.disconnect();

    }

    ros::spin() ;

    return 0 ;

}
