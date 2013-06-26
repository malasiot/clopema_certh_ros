#ifndef __CAMERA_VIEWER_SERVER_H__
#define __CAMERA_VIEWER_SERVER_H__


#include <boost/function.hpp>
#include <boost/unordered_map.hpp>
#include <boost/scoped_ptr.hpp>
#include <boost/thread/thread.hpp>
#include <boost/thread/recursive_mutex.hpp>
#include <boost/signals.hpp>

#include <viz_helpers/CameraViewFeedback.h>
#include <ros/callback_queue.h>

namespace viz_helpers {

class CameraViewServer : boost::noncopyable
{
public:

    CameraViewServer( const std::string &topic_ns = "");
    ~CameraViewServer();


    // connect this signal to any callback functions to receive mouse click feedback
    boost::signal<void (unsigned int, unsigned int)> mouseClicked ;

private:

    void spinThread();

    void processFeedback( const viz_helpers::CameraViewFeedbackConstPtr& feedback );

    // topic namespace to use
    std::string topic_ns_;

    boost::recursive_mutex mutex_;
    boost::scoped_ptr<boost::thread> spin_thread_;
    ros::NodeHandle node_handle_;
    ros::CallbackQueue callback_queue_;
    volatile bool need_to_terminate_;

    ros::Publisher update_pub_;
    ros::Subscriber feedback_sub_;

    std::string server_id_;


};

}
#endif
