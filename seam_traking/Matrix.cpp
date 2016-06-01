#include "Matrix.h"
#include <math.h>
#define PI 3.14159265

double Matrix::Frobenius(cv::Mat A) {
  CV_Assert(A.size != 0);
  double frobenius;
  cv::Mat square_A;
  square_A = A.mul(A);
  frobenius = sqrt(cv::sum(square_A)[0]);
  return frobenius;
}

cv::Mat Matrix::Kron(cv::Mat mat1, cv::Mat mat2) {
  CV_Assert(mat1.size != 0 && mat2.size != 0);
  cv::Mat K(mat1.rows * mat2.rows, mat1.cols * mat2.cols, CV_64F);
  cv::Range rows, cols;
  for (int i = 0; i < mat1.rows; i++) {
    for (int j = 0; j < mat1.cols; j++) {
      rows = cv::Range(i*mat2.rows, (i + 1)*mat2.rows);
      cols = cv::Range(j*mat2.cols, (j + 1)*mat2.cols);
      K.rowRange(rows).colRange(cols) = mat1.at<double>(i, j) * mat2;
    }
  }
  return K;
}

cv::Mat Matrix::RotateX(double angle, bool is_degree) {
  cv::Mat rotate_x;
  rotate_x = cv::Mat_<double>::eye(3, 3);
  if (is_degree) {
    angle = angle * PI / 180;
  }
  rotate_x.at<double>(1, 1) = cos(angle);
  rotate_x.at<double>(2, 2) = cos(angle);
  rotate_x.at<double>(1, 2) = sin(angle);
  rotate_x.at<double>(2, 1) = -sin(angle);
  return rotate_x;
}

cv::Mat Matrix::RotateY(double angle, bool is_degree) {
  cv::Mat rotate_y;
  rotate_y = cv::Mat_<double>::eye(3, 3);
  if (is_degree) {
    angle = angle * PI / 180;
  }
  rotate_y.at<double>(0, 0) = cos(angle);
  rotate_y.at<double>(2, 2) = cos(angle);
  rotate_y.at<double>(0, 2) = -sin(angle);
  rotate_y.at<double>(2, 0) = sin(angle);
  return rotate_y;
}

cv::Mat Matrix::RotateZ(double angle, bool is_degree) {
  cv::Mat rotate_z;
  rotate_z = cv::Mat_<double>::eye(3, 3);
  if (is_degree) {
    angle = angle * PI / 180;
  }
  rotate_z.at<double>(0, 0) = cos(angle);
  rotate_z.at<double>(1, 1) = cos(angle);
  rotate_z.at<double>(0, 1) = sin(angle);
  rotate_z.at<double>(1, 0) = -sin(angle);
  return rotate_z;
}
