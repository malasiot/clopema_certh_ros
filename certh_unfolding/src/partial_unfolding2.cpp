#include "fold_detector/FoldDetector.h"
#include "RotateAndGrab.h"
#include <robot_helpers/Utils.h>

#include <highgui.h>

using namespace cv ;
using namespace Eigen ;
using namespace robot_helpers ;
using namespace std ;

class FoldDetectorAction: public RotateAndGrab
{
public:
    FoldDetectorAction(const string &arm): RotateAndGrab("xtion3", arm) {
        counter = 0 ;
    }

    void process(const Mat &clr, const Mat &depth, const image_geometry::PinholeCameraModel cm, const ros::Time &ts, Affine3d &tip_pose_in_camera_frame)
    {
        cout << counter++ << endl ;

        ros::Duration(0.1).sleep() ;
        cv::imwrite(str(boost::format("/tmp/cap_%d.png") % counter), clr) ;
    }

    int counter ;
};

int main(int argc, char **argv) {

    ros::init(argc, argv, "unfolding2");
    ros::NodeHandle nh;
    ros::Publisher marker_pub;
    marker_pub = nh.advertise<visualization_msgs::Marker>("/visualization_marker", 0);



    setServoPowerOff() ;

    return 0 ;
}
