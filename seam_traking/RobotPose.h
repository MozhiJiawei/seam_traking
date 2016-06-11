#ifndef _ROBOTPOSE_H__
#define _ROBOTPOSE_H__
#include <opencv2/opencv.hpp>

class RobotPose {
public:
  RobotPose(double x, double y, double z, double rotate_angle, 
      double sway_angle) :x_(x), y_(y), z_(z), rotate_angle_(rotate_angle), 
      sway_angle_(sway_angle){}

  cv::Mat GetBase2Robot();
  double x_; // mm
  double y_;
  double z_;
  double rotate_angle_; // -360 ~ 360 degree
  double sway_angle_; // -90 ~ 45 degree
};

#endif /*_ROBOTPOSE_H__*/