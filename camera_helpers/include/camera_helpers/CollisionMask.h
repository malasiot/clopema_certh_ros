#ifndef __COLLISION_MASK_H_
#define __COLLISION_MASK_H_

#include <cv.h>

namespace camera_helpers {

// render a black and white image designating occluded pixels (by the robot) in the current view of the camera 

cv::Mat getCollisionMask(const std::string &camera_id) ;

}

#endif
