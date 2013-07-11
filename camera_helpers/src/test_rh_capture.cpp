#include <highgui.h>
#include <camera_helpers/RobotHead.h>

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "rh_capture_example");

    cv::Mat im_l, im_r ;

    camera_helpers::grabRHImages(im_l, im_r) ;

    cv::imwrite("/tmp/stereo_L.png", im_l) ;
    cv::imwrite("/tmp/stereo_R.png", im_r) ;
}
