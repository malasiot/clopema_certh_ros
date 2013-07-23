#ifndef __COMMAND_PANEL_SERVER_H__
#define __COMMAND_PANEL_SERVER_H__

#include <ros/ros.h>
#include <boost/function.hpp>
#include <boost/unordered_map.hpp>
#include <boost/scoped_ptr.hpp>
#include <boost/thread/thread.hpp>
#include <boost/thread/recursive_mutex.hpp>
#include <boost/signals.hpp>

#include <viz_helpers/CommandPanelFeedback.h>
#include <ros/callback_queue.h>

namespace viz_helpers {

class CommandPanelServer : boost::noncopyable
{
public:

    CommandPanelServer( const std::string &topic_ns = "");
    ~CommandPanelServer();

    // add a command and associated callback
    void addCommand(const std::string &cmd, boost::function<void ()> cb) ;

    // start the server
    void start() ;


private:

    void spinThread();

    void processFeedback( const viz_helpers::CommandPanelFeedbackConstPtr& feedback );
    void connectCallback(const ros::SingleSubscriberPublisher& pub) ;

    // topic namespace to use
    std::string topic_ns_;

    boost::recursive_mutex mutex_;
    boost::scoped_ptr<boost::thread> spin_thread_;
    ros::NodeHandle node_handle_;
    ros::CallbackQueue callback_queue_;
    volatile bool need_to_terminate_;

    ros::Publisher command_pub_;
    ros::Subscriber feedback_sub_;

    std::string server_id_;
    std::map<std::string, boost::function<void ()> > callbacks ;

};

}
#endif
