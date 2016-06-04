#ifndef _HANDEYECALIBRATOR_H__
#define _HANDEYECALIBRATOR_H__

#include <opencv2/opencv.hpp>
#include <vector>
#include "CameraCalibrator.h"

class Camera;

struct RobotPose {
public:
  RobotPose(double x, double y, double z, double rotate_angle, 
      double sway_angle) :x_(x), y_(y), z_(z), rotate_angle_(rotate_angle), 
      sway_angle_(sway_angle){}

  double x_; // mm
  double y_;
  double z_;
  double rotate_angle_; // -360 ~ 360 degree
  double sway_angle_; // -90 ~ 45 degree
};

class HandEyeCalibrator {
public:
  HandEyeCalibrator(Camera* cam, cv::Size board_size, double square_size);
  std::vector<int> AddPosePair(std::vector<cv::Mat>& src, 
      std::vector<RobotPose> robot_input, bool is_board_reverse = false);

  void GenerateBaseToWorld(std::vector<RobotPose> robot_input, 
      std::vector<cv::Point3d> world_points);

  /* at least three PosePair are needed to calculate the pose between
   camera and robot.
   Save the result in cam_->robot_to_camera_ */
  double Calibrate();
  
private:
  struct PosePair {
    cv::Mat world_to_camera_;
    cv::Mat base_to_robot_;
  };
  Camera* cam_;
  cv::Size board_size_;
  double square_size_; // mm
  std::vector<PosePair> calib_pose_;
  cv::Mat base_to_world_;

  //Convert the robot input to pose: base --> robot tool
  cv::Mat ConvertRobotPose(RobotPose robot);
  // A1 * X = X * B1
  // A2 * X = X * B2
  cv::Mat SolveX(cv::Mat A1, cv::Mat B1, cv::Mat A2, cv::Mat B2);
};
#endif /*_HANDEYECALIBRATOR_H__*/ 
