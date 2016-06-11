#include "RobotPose.h"
#include "Matrix.h"

cv::Mat RobotPose::GetBase2Robot() {
  cv::Mat base_to_robot, tvec, rmat;
  cv::Vec<double,3> t(x_, y_, z_);
  rmat = Matrix::RotateX(-sway_angle_) * 
      Matrix::RotateZ(rotate_angle_) * Matrix::RotateZ(90);

  tvec = -rmat * cv::Mat(t).reshape(1, 3);
  base_to_robot = cv::Mat_<double>::eye(4, 4);
  rmat.copyTo(base_to_robot.rowRange(0, 3).colRange(0, 3));
  tvec.copyTo(base_to_robot.rowRange(0, 3).col(3));
  return base_to_robot;
}
