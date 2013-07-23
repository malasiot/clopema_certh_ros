#include "Commander.h"
#include <viz_helpers/CommandPanelServer.h>

using namespace std ;

namespace viz_helpers {

CommandPanelServer::CommandPanelServer( const std::string &topic_ns )
{

    node_handle_.setCallbackQueue( &callback_queue_ );

    std::string feedback_topic = topic_ns + "command_panel/feedback";
    std::string command_topic = topic_ns + "command_panel/command";

    feedback_sub_ = node_handle_.subscribe( feedback_topic, 1, &CommandPanelServer::processFeedback, this );
    command_pub_ = node_handle_.advertise<viz_helpers::CommandPanelFeedback>( command_topic, 1, boost::bind(&CommandPanelServer::connectCallback,this, _1) );

    need_to_terminate_ = false ;

}

void CommandPanelServer::start()
{
    spin_thread_.reset( new boost::thread(boost::bind(&CommandPanelServer::spinThread, this)) );
}

void CommandPanelServer::connectCallback(const ros::SingleSubscriberPublisher& pub)
{
    boost::recursive_mutex::scoped_lock lock( mutex_ );

    map<string, boost::function<void ()> >::const_iterator it = callbacks.begin() ;

    for( ; it != callbacks.end() ; ++it )
    {
        CommandPanelFeedback msg ;
        msg.command = (*it).first ;

        pub.publish(msg) ;
    }

}

CommandPanelServer::~CommandPanelServer()
{
    if (spin_thread_.get())
    {
        need_to_terminate_ = true;
        spin_thread_->join();
    }

    if ( node_handle_.ok() )
    {

    }
}


void CommandPanelServer::spinThread()
{
    while (node_handle_.ok())
    {
        if (need_to_terminate_)
        {
            break;
        }
        callback_queue_.callAvailable(ros::WallDuration(0.033f));
    }
}

void CommandPanelServer::processFeedback( const CommandPanelFeedbackConstPtr& feedback )
{
    boost::recursive_mutex::scoped_lock lock( mutex_ );

    map<string, boost::function<void ()> >::const_iterator it = callbacks.find(feedback->command) ;

    if ( it == callbacks.end() )
    {
        ROS_ERROR("Undefined callback for command %s", feedback->command.c_str()) ;
        return ;
    }

    ((*it).second)() ;

}

void CommandPanelServer::addCommand(const std::string &cmd, boost::function<void ()> cb)
{
    boost::recursive_mutex::scoped_lock lock( mutex_ );

    callbacks[cmd] = cb ;

}

}
