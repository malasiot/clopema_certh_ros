#ifndef __CV_HELPERS_H__
#define __CV_HELPERS_H__

#include <cv.h>

namespace certh_libs {

// compute pixels on the line defined by points p1, p2
void getScanLine(const cv::Point &p1, const cv::Point &p2, std::vector<cv::Point> &pts) ;

// compute image gradient of a color image. For each pixel the maximum magnitude gradient from the 3 color bands
// is selected. Gradient magnitude is also thresholded using the provided threshold.

void computeGradient(const cv::Mat &clr, cv::Mat &mag, cv::Mat &ang, double gradMagThreshold) ;
void computeGradientField(const cv::Mat &clr, cv::Mat &gx, cv::Mat &gy, double gradMagThreshold) ;


}


#endif


