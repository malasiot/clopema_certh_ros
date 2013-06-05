#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>
#include <nodelet/loader.h>
#include <ros/ros.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

// OpenCV includes
#include <opencv2/highgui/highgui.hpp>
#include <boost/thread.hpp>
#include <boost/format.hpp>


#include <Eigen/Core>
   
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/ros/conversions.h>

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef Eigen::Vector3f Vec3 ;
typedef Eigen::Matrix3f Matrix3 ;

using namespace message_filters ;
using namespace sensor_msgs ;
using namespace std ;
namespace enc = sensor_msgs::image_encodings;

cv_bridge::CvImagePtr tmp_rgb, tmp_depth ;
PointCloud2::ConstPtr tmp_cloud;

boost::mutex lock ;

// Timestamps
ros::Time stamp_rgb;
ros::Time stamp_depth;
ros::Time stamp_cloud;

#define WINDOW "capture_view"

void input_callback(const ImageConstPtr& rgb, const ImageConstPtr& depth, const PointCloud2::ConstPtr cloud)
{
    // Store current images
    try {

        tmp_rgb = cv_bridge::toCvCopy(rgb, enc::BGR8);
        tmp_depth = cv_bridge::toCvCopy(depth, "");


         cout << "Grabbing images" << endl;

    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    // Store current point cloud
    tmp_cloud = cloud;


    PointCloud pc ;
    pcl::fromROSMsg (*tmp_cloud, pc) ;


    // Save stamps for info
    stamp_rgb = rgb->header.stamp;
    stamp_depth = depth->header.stamp;
    stamp_cloud = cloud->header.stamp;

    // Show rgb image to user
    //cv::imshow(WINDOW, tmp_rgb->image);
    //cv::waitKey(1);

    cv::Mat img(tmp_depth->image.rows, tmp_depth->image.cols, CV_16UC1);
    cv::Mat f_img(tmp_depth->image.rows, tmp_depth->image.cols, CV_32FC1);
    /*
    for(int i = 0; i < tmp_depth->image.rows; i++)
    {
        unsigned short* Di = tmp_depth->image.ptr<unsigned short>(i);
        unsigned short* Ii = img.ptr<unsigned short>(i);
        for(int j = 0; j < tmp_depth->image.cols; j++)
        {
            Ii[j] = Di[j];
        }
    }*/
    unsigned short* data = tmp_depth->image.ptr<unsigned short>(0);
    unsigned short max_val = 0;
    for(int rows = 0; rows<480; rows++){
        for(int cols = 0; cols<640; cols++){
            unsigned short val = data[0];
            if(val > max_val)
                max_val = val;
            img.at<unsigned short>(rows, cols) = val;
            f_img.at<float>(rows, cols) = (float)val;
            data++;
        }
    }
    f_img = f_img/(float)max_val;
    cv::imshow(WINDOW, f_img);
    cv::waitKey(1);

}

void grab_images()
{
    cout<< "yeahhh";
    for( int i=0 ; i<100 ;  )
    {

        if ( !tmp_rgb ) continue ;
        cout<< "done";
        string fileNameC = "/home/clopema/akarg" + str(boost::format("image%03d_c.png") % i) ;
        string fileNameD = "/home/clopema/akarg" + str(boost::format("image%03d_d.png") % i) ;

        cout << "Saving images: " << i;

        cv::imwrite(fileNameC, tmp_rgb->image) ;
        cv::imwrite(fileNameD, tmp_depth->image) ;

        ++i ;
        ros::Duration(1.0).sleep() ;
    }



}

extern bool findLowestPoint(const PointCloud &depth, const Vec3 &orig, const Vec3 &base, float apperture,  Vec3 &p, Vec3 &n) ;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "openni_example");
    ros::NodeHandle nh ;

    nodelet::Loader manager(true); // Bring up manager ROS API

    nodelet::M_string remappings;
    nodelet::V_string my_argv;

    bool res = manager.load(ros::this_node::getName(), "openni_camera/driver", remappings, my_argv);

    // Subscribe to rgb and depth streams
      Subscriber<sensor_msgs::Image> rgb_sub(nh, "xtion2/rgb/image_raw", 1);
      Subscriber<sensor_msgs::Image> depth_sub(nh, "xtion2/depth/image_raw", 1);
      Subscriber<PointCloud2> cloud_sub(nh, "xtion2/depth/points", 1);

     Synchronizer<sync_policies::ApproximateTime<Image, Image, PointCloud2> > sync(sync_policies::ApproximateTime<Image, Image, PointCloud2>(10), rgb_sub, depth_sub, cloud_sub);
     sync.registerCallback(boost::bind(&input_callback, _1, _2, _3));
     /*
      Synchronizer<sync_policies::ApproximateTime<Image, Image> >
      sync(sync_policies::ApproximateTime<Image, Image>(10), rgb_sub, depth_sub);
      sync.registerCallback(boost::bind(&input_callback, _1, _2));
*/
      // Create window to display actual images
      cv::namedWindow(WINDOW);

      boost::thread t (grab_images) ;
      // Spin ros node
      ros::spin();
	
}
