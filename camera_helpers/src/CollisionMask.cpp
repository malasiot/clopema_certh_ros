#include <ros/ros.h>
#include <camera_helpers/CollisionMask.h>
#include <camera_helpers/OffscreenRenderService.h>
#include <cv_bridge/cv_bridge.h>

using namespace cv ;
using namespace std ;

namespace camera_helpers {


Mat getCollisionMask(const std::string &camera_id)
{
    if ( !ros::service::waitForService(camera_id + "/offscreen_render/render", ros::Duration(5.0)) )
    {
        ROS_ERROR("Cannot connect to offscreen renderer service") ;
        return Mat() ;
    }

    camera_helpers::OffscreenRenderService::Request req ;
    camera_helpers::OffscreenRenderService::Response res ;

    ros::service::call(camera_id + "/offscreen_render/render", req, res) ;

    cv_bridge::CvImagePtr img = cv_bridge::toCvCopy(res.mask, "mono8") ;

    return img->image ;
}


}
