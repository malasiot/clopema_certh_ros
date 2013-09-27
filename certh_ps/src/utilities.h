#ifndef UTILITIES_H
#define UTILITIES_H

#include <iostream>
#include <stdlib.h>
#include <opencv2/opencv.hpp>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>


void photometricStereo(cv::Mat &Lights, cv::Mat *IgrFF, int w, int lowbound, int upbound, int numImages, cv::Mat &Nx, cv::Mat &Ny, cv::Mat &Nz, cv::Mat &alb_gr);
void init_port(int *fd, unsigned int baud);
cv::Mat playCamera(cv::VideoCapture *cap, int serialPort, cv::Rect roi, const char *ledNumber, std::string whichImages);
cv::Mat flatFielding(int w, cv::Mat FF_Image, cv::Mat inImage, std::string How);
cv::Mat linRegress(cv::Mat x1, cv::Mat x2, cv::Mat y, double n);
void quadrantShift(int w, cv::Mat &wx, cv::Mat &wy);
cv::Mat psCloud(int w, cv::Mat Nx, cv::Mat Ny, cv::Mat Nz);
cv::Scalar focusMetric(cv::Mat frame, cv::Rect roiFM);

#endif // UTILITIES_H

