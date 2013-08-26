#ifndef __CALIBRATION_H__
#define __CALIBRATION_H__

#include <Eigen/Geometry>
#include <vector>

#include <cv.h>

// Various methods for solving the hande-eye calibration problem AX=XB for motions A, B
// If refine = true a non-linear optimization is performed to refine the solution

enum HandEyeMethod { Horaud, Tsai, DualQuat } ;

bool solveHandEyeFixed(const std::vector<Eigen::Affine3d> &gripper_to_base, const std::vector<Eigen::Affine3d> &target_to_sensor,
                  HandEyeMethod method, bool refine, Eigen::Affine3d &sensor_to_base ) ;

bool solveHandEyeMoving(const std::vector<Eigen::Affine3d> &gripper_to_base, const std::vector<Eigen::Affine3d> &target_to_sensor,
                  HandEyeMethod method, bool refine, Eigen::Affine3d &sensor_to_gripper ) ;

// Method by Horaud and Dornaika

bool solveHandEyeLinearHD(const std::vector<Eigen::Affine3d> &A, const std::vector<Eigen::Affine3d> &B, Eigen::Affine3d &X ) ;

// Method by R. Tsai

bool solveHandEyeLinearTsai(const std::vector<Eigen::Affine3d> &A, const std::vector<Eigen::Affine3d> &B, Eigen::Affine3d &X ) ;

// Method by K. Danielidis

bool solveHandEyeLinearDualQuaternion(const std::vector<Eigen::Affine3d> &A, const std::vector<Eigen::Affine3d> &B, Eigen::Affine3d &X ) ;

// minimize the frobenius norm of residual errors in rotation and translation (according to Dornaika and Horaud)
// uses numerical differentiation for computing Jacobians

bool solveHandEyeNonLinear(const std::vector<Eigen::Affine3d> &A, const std::vector<Eigen::Affine3d> &B, Eigen::Affine3d &X ) ;

// use calibration images and robot poses to determine the transformations between gripper and base and between calibration target and sensor

void find_target_motions(const std::string &filePrefix, const std::string &dataFolder, const cv::Size boardSize, const double squareSize,
                         cv::Mat &cameraMatrix, bool useDepth,
                           std::vector<Eigen::Affine3d> &gripper_to_base, std::vector<Eigen::Affine3d> &target_to_sensor ) ;

#endif
