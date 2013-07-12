#ifndef __ROBOT_HEAD_H__
#define __ROBOT_HEAD_H__

#include <cv.h>
#include <ros/ros.h>
#include <image_geometry/stereo_camera_model.h>

namespace camera_helpers {

/*
    capture stereo images:

    preview: if set to true low resolution images will be captured
    msec: if the acquisition is not completed within the interval the procedure returns false. By default it waits infinitely
*/

bool grabRHImages(cv::Mat &left, cv::Mat &right, image_geometry::StereoCameraModel &cm, bool preview = false, unsigned int delay_msec = 0) ;


}


#endif
