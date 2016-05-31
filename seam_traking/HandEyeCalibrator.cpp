#include "HandEyeCalibrator.h"
#include "Camera.h"

#define Test 1;

HandEyeCalibrator::HandEyeCalibrator(Camera* cam, cv::Size board_size, 
  double square_size) :cam_(cam), board_size_(board_size), 
  square_size_(square_size) {}

std::vector<int> HandEyeCalibrator::AddPosePair(std::vector<cv::Mat>& src, 
    std::vector<RobotPose>, bool is_board_reverse) {


  return std::vector<int>();
}

double HandEyeCalibrator::Calibrate() {
  cv::Mat Camera_ij, Camera_jk, Robot_ij, Robot_jk;
  // Test using Test data
  double A1[4][4] = { -0.989992, -0.141120, 0, 0,
    0.141120, -0.989992, 0, 0,
    0, 0, 1, 0, 
    0, 0, 0, 1 };

  double B1[4][4] = { -0.989992, -0.138307, 0.028036, -26.9559,
    0.138307, -0.911449, 0.387470, -96.1332,
    -0.028036, 0.387470, 0.921456, 19.4872,
    0, 0, 0, 1 };

  double A2[4][4] = { 0.070737, 0, 0.997495, -400,
    0, 1, 0, 0,
    -0.997495, 0, 0.070737, 400,
    0, 0, 0, 1 };

  double B2[4][4] = { 0.070737, 0.198172, 0.977612, -309.543,
    -0.198172, 0.963323, -0.180936, 59.0244,
    -0.977612, -0.180936, 0.107415, 291.177,
    0, 0, 0, 1 };

  Camera_ij = cv::Mat(4, 4, CV_64F, &A1);
  Robot_ij = cv::Mat(4, 4, CV_64F, &B1);
  Camera_jk = cv::Mat(4, 4, CV_64F, &A2);
  Robot_jk = cv::Mat(4, 4, CV_64F, &B2);
  std::cout << Camera_ij << std::endl;

  //cv::Mat A, B;
  //A = cv::Mat_<double>::eye(4,4);
  //double b[2][2] = { 1, -1, -1, 1 };
  //B = cv::Mat(2, 2, CV_64F, &b);
  //std::cout << A << std::endl;
  //std::cout << B << std::endl;
  //std::cout << Kron(A, B) << std::endl;
  std::cout << SolveX(Camera_ij, Robot_ij, Camera_jk, Robot_jk) << std::endl;
  return 0.0;
}

cv::Mat HandEyeCalibrator::ConvertRobotPose() {
  return cv::Mat();
 };

cv::Mat HandEyeCalibrator::Kron(cv::Mat A, cv::Mat B) {
  cv::Mat K(A.rows * B.rows, A.cols * B.cols, CV_64F);
  for (int i = 0; i < A.rows; i++) {
    for (int j = 0; j < A.cols; j++) {
      K.rowRange(i*B.rows, (i + 1)*B.rows).colRange(j*B.cols, (j + 1)*B.cols) =
        A.at<double>(i, j) * B;
    }
  }
  return K;
}

cv::Mat HandEyeCalibrator::SolveX(cv::Mat A1, cv::Mat B1,
  cv::Mat A2, cv::Mat B2) {

  cv::Mat RA1, tA1, RB1, tB1, RA2, tA2, RB2, tB2;
  cv::Mat w, u, vt, v;
  cv::Mat C(18, 9, CV_64F);
  cv::Mat M1, M2, N1, N2, tx, R, X;
  cv::Mat A(6, 4, CV_64F);
  cv::Mat b(6, 1, CV_64F);
  A1.rowRange(0, 3).colRange(0, 3).copyTo(RA1);
  A1.rowRange(0, 3).col(3).copyTo(tA1);
  B1.rowRange(0, 3).colRange(0, 3).copyTo(RB1);
  B1.rowRange(0, 3).col(3).copyTo(tB1);
  A2.rowRange(0, 3).colRange(0, 3).copyTo(RA2);
  A2.rowRange(0, 3).col(3).copyTo(tA2);
  B2.rowRange(0, 3).colRange(0, 3).copyTo(RB2);
  B2.rowRange(0, 3).col(3).copyTo(tB2);
  //std::cout << RA1 << std::endl;
  //std::cout << tA1 << std::endl;
  //std::cout << RB1 << std::endl;
  //std::cout << tB1 << std::endl;
  //std::cout << RA2 << std::endl;
  //std::cout << tA2 << std::endl;
  //std::cout << RB2 << std::endl;
  //std::cout << tB2 << std::endl;

  C.rowRange(0, 9) = cv::Mat_<double>::eye(9, 9) - Kron(RA1, RB1);
  C.rowRange(9, 18) = cv::Mat_<double>::eye(9, 9) - Kron(RA2, RB2);
  cv::SVD::compute(C, w, u, vt);
  //std::cout << C << std::endl;
  //std::cout << vt << std::endl;

  v = -vt.row(8).t();
  //std::cout << v << std::endl;
  //std::cout << v.reshape(1, 3) << std::endl;

  M1 = Kron(cv::Mat_<double>::eye(3, 3), tB1.t()) * v;
  M2 = Kron(cv::Mat_<double>::eye(3, 3), tB2.t()) * v;
  N1 = cv::Mat_<double>::eye(3, 3) - RA1;
  N2 = cv::Mat_<double>::eye(3, 3) - RA2;
  M1.copyTo(A.rowRange(0, 3).col(0));
  N1.copyTo(A.rowRange(0, 3).colRange(1, 4));
  M2.copyTo(A.rowRange(3, 6).col(0));
  N2.copyTo(A.rowRange(3, 6).colRange(1, 4));
  tA1.copyTo(b.rowRange(0, 3));
  tA2.copyTo(b.rowRange(3, 6));

  tx = (A.t() * A).inv() * (A.t() * b);
  R = v.reshape(1, 3).t() * tx.at<double>(0, 0);
  X = cv::Mat_<double>::zeros(4, 4);
  R.copyTo(X.rowRange(0, 3).colRange(0, 3));
  tx.rowRange(1, 4).copyTo(X.rowRange(0, 3).col(3));
  X.at<double>(3, 3) = 1;
  return X;
}
