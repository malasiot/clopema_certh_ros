#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <image_geometry/pinhole_camera_model.h>
#include <cv_bridge/cv_bridge.h>
#include <cv_bridge/CvBridge.h>

#include <boost/thread.hpp>
#include <deque>

namespace enc = sensor_msgs::image_encodings;

using namespace std ;

namespace camera_helpers {




class DepthFilter {
public:
    DepthFilter(int nFrames_): nFrames(nFrames_) {}

    cv::Mat getFiltered(const cv::Mat &im)
    {
        if ( frames.size() == nFrames ) frames.pop_front() ;
        frames.push_back(im) ;

        int w = im.cols, h = im.rows ;
        cv::Mat_<uint> cnt = cv::Mat_<uint>::zeros(im.size()) ;
        cv::Mat_<float> res_ = cv::Mat_<float>::zeros(im.size()) ;

        for(int k=0 ; k<frames.size() ; k++ )
        {
            cv::Mat_<ushort> buf = frames[k] ;

            for(int i=0 ; i<h ; i++)
                for(int j=0 ; j<w ; j++)
                {
                    if ( buf[i][j] != 0 )
                    {
                        res_[i][j] += buf[i][j] ;
                        cnt[i][j] ++ ;
                    }
                }
        }

        cv::Mat_<ushort> res(im.size()) ;

        for(int i=0 ; i<h ; i++)
            for(int j=0 ; j<w ; j++)
            {
                if ( cnt[i][j] == 0 ) res[i][j] = 0 ;
                else res[i][j] = res_[i][j]/cnt[i][j] ;
            }

        return res ;
    }

private:
    deque<cv::Mat> frames ;
    int nFrames ;
};

class DepthFilterNodelet : public nodelet::Nodelet
{
    // Subscriptions
    boost::shared_ptr<image_transport::ImageTransport> it_;
    image_transport::Subscriber sub_depth_;
    int queue_size_;

    // Publications
    boost::mutex connect_mutex_;
    ros::Publisher pub_depth_;
    DepthFilter *filter_ ;

    virtual void onInit();

    void connectCb();

    void depthCb(const sensor_msgs::ImageConstPtr& depth_msg);
};

void DepthFilterNodelet::onInit()
{
    ros::NodeHandle& nh         = getNodeHandle();
    ros::NodeHandle& private_nh = getPrivateNodeHandle();

    int nFrames = 10;
    private_nh.getParam("num_frames", nFrames) ;

    filter_ = new DepthFilter(nFrames) ;

    it_.reset(new image_transport::ImageTransport(nh));

    // Read parameters
    private_nh.param("queue_size", queue_size_, 5);

    // Monitor whether anyone is subscribed to the output
    ros::SubscriberStatusCallback connect_cb = boost::bind(&DepthFilterNodelet::connectCb, this);
    // Make sure we don't enter connectCb() between advertising and assigning to pub_point_cloud_
    boost::lock_guard<boost::mutex> lock(connect_mutex_);
    pub_depth_ = nh.advertise<sensor_msgs::Image>("image_filtered", 1, connect_cb, connect_cb);

}

// Handles (un)subscribing when clients (un)subscribe
void DepthFilterNodelet::connectCb()
{
    boost::lock_guard<boost::mutex> lock(connect_mutex_);

    if (pub_depth_.getNumSubscribers() == 0)
    {
        sub_depth_.shutdown();
    }
    else if (!sub_depth_)
    {
        image_transport::TransportHints hints("raw", ros::TransportHints(), getPrivateNodeHandle());
        sub_depth_ = it_->subscribe("image_rect", queue_size_, &DepthFilterNodelet::depthCb, this, hints);

    }
}

void DepthFilterNodelet::depthCb(const sensor_msgs::ImageConstPtr& depth_msg)
{
    boost::lock_guard<boost::mutex> lock(connect_mutex_);

    sensor_msgs::ImagePtr depth_out_msg ;

    cv_bridge::CvImagePtr depth_ = cv_bridge::toCvCopy(depth_msg, "");

    cv::Mat filtered = filter_->getFiltered(depth_->image) ;

    IplImage ipl_ = filtered ;
    depth_out_msg = sensor_msgs::CvBridge::cvToImgMsg(&ipl_, "passthrough");

    pub_depth_.publish (depth_out_msg);
}

}
// Register as nodelet
#include <pluginlib/class_list_macros.h>
PLUGINLIB_DECLARE_CLASS (camera_helpers, depth_filter, camera_helpers::DepthFilterNodelet, nodelet::Nodelet)
