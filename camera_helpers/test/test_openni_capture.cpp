#include <camera_helpers/OpenNICapture.h>

#include <string>
#include <highgui.h>
#include <pcl/io/pcd_io.h>

void grabpc(camera_helpers::OpenNICapturePointCloud *grabber)
{

    pcl::PointCloud<pcl::PointXYZRGB> cloud ;
    ros::Time ts ;

    std::cout << "start grabbing PC" << std::endl ;
    for(int i=0 ; i<10 ; i++ )
    {
        grabber->grab(cloud, ts) ;

        pcl::io::savePCDFileASCII(str(boost::format("/tmp/cloud_%03d.pcd") % i), cloud) ;

      //  ros::Duration(0.3).sleep() ;
    }
    std::cout << "finished PC" << std::endl ;

    grabber->disconnect() ;

}


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "openni_capture_example");

 //   camera_helpers::OpenNICapturePointCloud grabber2("xtion3") ;

    //grabber2.connect(boost::bind( &grabpc, &grabber2 )) ;

    camera_helpers::OpenNICaptureRGBD grabber("xtion3") ;

    image_geometry::PinholeCameraModel camera ;

    if ( grabber.connect() )
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
