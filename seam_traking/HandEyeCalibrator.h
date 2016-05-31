#ifndef _HANDEYECALIBRATOR_H__
#define _HANDEYECALIBRATOR_H__

#include <opencv2/opencv.hpp>
#include <vector>
#include "CameraCalibrator.h"

class Camera;

struct RobotPose {
  double x_; // mm
  double y_;
  double z_;
  double spin_angle_; // -360 ~ 360 degree
  double sway_angle_; // -90 ~ 45 degree
};

class HandEyeCalibrator {
public:
  HandEyeCalibrator(Camera* cam, cv::Size board_size, double square_size);
  std::vector<int> AddPosePair(std::vector<cv::Mat>& src, 
      std::vector<RobotPose>, bool is_board_reverse = false);

  double Calibrate();
  
private:
  struct PosePair {
    cv::Mat camer_pose_;
    cv::Mat robot_pose_;
  };
  Camera* cam_;
  cv::Size board_size_;
  double square_size_; // unit: mm
  std::vector<PosePair> calib_pose_;

  cv::Mat ConvertRobotPose();
  cv::Mat Kron(cv::Mat mat1, cv::Mat mat2);
  // A1 * X = X * B1
  // A2 * X = X * B2
  cv::Mat SolveX(cv::Mat A1, cv::Mat B1, cv::Mat A2, cv::Mat B2);
};
#endif /*_HANDEYECALIBRATOR_H__*/ 

