#ifndef __CAMERA_VIEWER_SERVER_H__
#define __CAMERA_VIEWER_SERVER_H__


#include <boost/function.hpp>
#include <boost/unordered_map.hpp>
#include <boost/scoped_ptr.hpp>
#include <boost/thread/thread.hpp>
#include <boost/thread/recursive_mutex.hpp>

namespace viz_helpers {

class CameraViewServer : boost::noncopyable
{
public:

    typedef boost::function<void (int, int)> MouseClickCallback ;


    CameraViewServer( const std::string &topic_ns = "");

    /// Destruction of the interface will lead to all managed markers being cleared.
    ~CameraViewServer();

    void setMouseClickCallback(MouseClickCallback cb) ;

private:

  // main loop when spinning our own thread
  // - process callbacks in our callback queue
  // - process pending goals
  void spinThread();

  void processFeedback( const viz_helpers::CameraViewFeedbackConstPtr& feedback );

  // topic namespace to use
  std::string topic_ns_;

  boost::recursive_mutex mutex_;

  // these are needed when spinning up a dedicated thread
  boost::scoped_ptr<boost::thread> spin_thread_;
  ros::NodeHandle node_handle_;
  ros::CallbackQueue callback_queue_;
  volatile bool need_to_terminate_;

  ros::Publisher update_pub_;
  ros::Subscriber feedback_sub_;

  std::string server_id_;

  MouseClickCallback mcCb ;
};

}
#endif
