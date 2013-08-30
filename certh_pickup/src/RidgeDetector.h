#ifndef __RIDGE_DETECTOR_H__
#define __RIDGE_DETECTOR_H__

#include <cv.h>

class RidgeDetector {
public:

    struct Parameters {
        double responseThreshold ; // ridge detector response threshold
        double minScale, maxScale, scaleStep ; // for multi-scale search

        Parameters():
            responseThreshold(10.0),
            minScale(4.0), maxScale(8.0), scaleStep(0.25) {}

    };

    struct GraspCandidate {
        double x, y ; // position on the image
        double width ;   // width of the ridge
        double alpha ;   // angle
        double strength ;
    };


    RidgeDetector() {}
    RidgeDetector(const Parameters &params_): params(params_) {}

    // run detector to obtain a list of grasp candidates sorted by strength

    bool detect(const cv::Mat &src, std::vector<GraspCandidate> &cand) ;

    // draw candidates on a color image

    void draw(cv::Mat &clr, const std::vector<GraspCandidate> &cand) ;


protected:

    // detect ridges on the depth map (Koller et. al, Multiscale Detection of Curvilinear Structures in 2D and 3D Image data)
    // The result is a binary map designating ridge points
    // The images alpha and sigma are the principal curvature orientation of the ridge and sigma is proportional ro the width of the ridge

    cv::Mat detect(const cv::Mat &src, cv::Mat &alpha, cv::Mat &sigma, cv::Mat &ridges) ;

    // select candidates by a non-maximum supression procedure for features with similar orientation

    void findCandidates(const cv::Mat &src, const cv::Mat &resp, const cv::Mat &ridges,
                                const cv::Mat &alpha, const cv::Mat &sigma, std::vector<GraspCandidate> &cand) ;



private:

    Parameters params ;

};
















#endif
