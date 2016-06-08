#ifndef _MATRIX_H__
#define _MATRIX_H__
#include <opencv2/opencv.hpp>

class Matrix {
public:
  static double Frobenius(cv::Mat A);
  static cv::Mat Kron(cv::Mat mat1, cv::Mat mat2);
  static double GetLineAngle(double x_diff, double y_diff,
      bool is_degree = true);
  
  static cv::Mat RotateX(double angle, int is_33 = 3, bool is_degree = true);
  static cv::Mat RotateY(double angle, int is_33 = 3, bool is_degree = true);
  static cv::Mat RotateZ(double angle, int is_33 = 3, bool is_degree = true);
};
#endif /*_MATRIX_H__*/