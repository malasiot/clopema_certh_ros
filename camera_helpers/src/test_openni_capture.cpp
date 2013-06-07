#include <camera_helpers/OpenNICapture.h>

#include <string>
#include <highgui.h>
#include <pcl/io/pcd_io.h>

void grab(camera_helpers::OpenNICaptureRGBD *grabber)
{

    cv::Mat clr, depth ;
    ros::Time ts ;

    std::cout << "start grabbing" << std::endl ;
    for(int i=0 ; i<10 ; i++ )
    {
        grabber->grab(clr, depth, ts) ;

        cv::imwrite(str(boost::format("/home/malasiot/tmp/rgb_%03d.png") % i), clr) ;
        cv::imwrite(str(boost::format("/home/malasiot/tmp/depth_%03d.png") % i), depth) ;

        ros::Duration(1).sleep() ;
    }
    std::cout << "finished" << std::endl ;

    grabber->disconnect() ;

}

void grabpc(camera_helpers::OpenNICapturePointCloud *grabber)
{

    pcl::PointCloud<pcl::PointXYZRGB> cloud ;
    ros::Time ts ;

    std::cout << "start grabbing PC" << std::endl ;
    for(int i=0 ; i<10 ; i++ )
    {
        grabber->grab(cloud, ts) ;

        pcl::io::savePCDFileASCII(str(boost::format("/home/malasiot/tmp/cloud_%03d.pcd") % i), cloud) ;

      //  ros::Duration(0.3).sleep() ;
    }
    std::cout << "finished PC" << std::endl ;

    grabber->disconnect() ;

}


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "openni_capture_example");

    camera_helpers::OpenNICaptureRGBD grabber("xtion2") ;

    grabber.connect(grab) ;

    camera_helpers::OpenNICapturePointCloud grabber2("xtion2") ;

    grabber2.connect(grabpc) ;

    ros::spin() ;

    return 0 ;

}
