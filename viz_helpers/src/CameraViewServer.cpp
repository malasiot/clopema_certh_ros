#include "CameraView.h"
#include <viz_helpers/CameraViewServer.h>

using namespace std ;

namespace viz_helpers {

CameraViewServer::CameraViewServer( const std::string &topic_ns )
{

    node_handle_.setCallbackQueue( &callback_queue_ );

    std::string feedback_topic = topic_ns + "camera_viewer/feedback";

    feedback_sub_ = node_handle_.subscribe( feedback_topic, 100, &CameraViewServer::processFeedback, this );

    need_to_terminate_ = false ;
    spin_thread_.reset( new boost::thread(boost::bind(&CameraViewServer::spinThread, this)) );

}


CameraViewServer::~CameraViewServer()
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


void CameraViewServer::spinThread()
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

void CameraViewServer::processFeedback( const CameraViewFeedbackConstPtr& feedback )
{
    boost::recursive_mutex::scoped_lock lock( mutex_ );

    mouseClicked(feedback->mouse_point.x, feedback->mouse_point.y) ;
}

}
