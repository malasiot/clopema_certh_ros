#ifndef __RIDGE_DETECTOR_H__
#define __RIDGE_DETECTOR_H__

#include <cv.h>

class RidgeDetector {
public:

    struct Parameters {
        double responseThreshold ; // ridge detector response threshold
        double minScale, maxScale, scaleStep ; // for multi-scale search

        Parameters():
            responseThreshold(0.0),
            minScale(4.0), maxScale(12.0), scaleStep(0.25) {}

    };


    RidgeDetector() {} ;
    RidgeDetector(const Parameters &params_): params(params_) {}

    // detect ridges on the depth map (Koller et. al, Multiscale Detection of Curvilinear Structures in 2D and 3D Image data
    // The result is a binary map designating ridge points
    // The images alpha and sigma are the principal curvature orientation of the ridge and sigma is proportional ro the width of the ridge

    void detect(const cv::Mat &src, cv::Mat &alpha, cv::Mat &sigma, cv::Mat &ridges) ;

    // this compute the area of the ridge that lies inside a virtual gripper

    cv::Mat computeGraspability(const cv::Mat &src, const cv::Mat &ridges,
                                const cv::Mat &alpha, const cv::Mat &sigma,
                                double gripperOpenning, double gripperWidth) ;

public:


private:

    Parameters params ;



};
















#endif
