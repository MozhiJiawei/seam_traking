#include "HandEyeCalibrator.h"
#include <math.h>
#include "Camera.h"

#define PI 3.14159265
#define Test 1;

HandEyeCalibrator::HandEyeCalibrator(Camera* cam, cv::Size board_size, 
  double square_size) :cam_(cam), board_size_(board_size), 
  square_size_(square_size) {}

std::vector<int> HandEyeCalibrator::AddPosePair(std::vector<cv::Mat>& src, 
    std::vector<RobotPose> robot_input, bool is_board_reverse) {

  if (src.size() != robot_input.size()) {
    // error and return
  }
  PosePair pose_pair;
  std::vector<int> error_pose_pair;
  for (int i = 0; i < src.size(); ++i) {
    pose_pair.world_to_camera_ = cam_->FindChessboardPose(src[i], 
        board_size_, square_size_, is_board_reverse);

    pose_pair.base_to_robot_ = ConvertRobotPose(robot_input[i]);
    if (abs(pose_pair.world_to_camera_.at<double>(3, 3) - 1) < 0.00001 &&
        abs(pose_pair.base_to_robot_.at<double>(3, 3) - 1) < 0.00001) {

      calib_pose_.push_back(pose_pair);
    } else {
      error_pose_pair.push_back(i + 1);
    }
  }
  return error_pose_pair;
}

double HandEyeCalibrator::Calibrate() {
  cv::Mat camera_ij, camera_jk, robot_ij, robot_jk, X;
  double error_frobenius;
  // Test using Test data
  double A1[4][4] = { -0.989992, -0.141120, 0, 0,
    0.141120, -0.989992, 0, 0,
    0, 0, 1, 0, 0, 0, 0, 1 };

  double B1[4][4] = { -0.989992, -0.138307, 0.028036, -26.9559,
    0.138307, -0.911449, 0.387470, -96.1332,
    -0.028036, 0.387470, 0.921456, 19.4872,
    0, 0, 0, 1 };

  double A2[4][4] = { 0.070737, 0, 0.997495, -400,
    0, 1, 0, 0, -0.997495, 0, 0.070737, 400,
    0, 0, 0, 1 };

  double B2[4][4] = { 0.070737, 0.198172, 0.977612, -309.543,
    -0.198172, 0.963323, -0.180936, 59.0244,
    -0.977612, -0.180936, 0.107415, 291.177,
    0, 0, 0, 1 };

  camera_ij = cv::Mat(4, 4, CV_64F, &A1);
  robot_ij = cv::Mat(4, 4, CV_64F, &B1);
  camera_jk = cv::Mat(4, 4, CV_64F, &A2);
  robot_jk = cv::Mat(4, 4, CV_64F, &B2);

  X = SolveX(camera_ij, robot_ij, camera_jk, robot_jk);
  error_frobenius = Frobenius(camera_ij * X - X * robot_ij) + 
      Frobenius(camera_jk * X - X * robot_jk);
  
  std::cout << error_frobenius << std::endl;
  RobotPose ref(773.40, -638.90, -349.99, -89.98, 0),
    p1(773.40, -463.90, -303.11, -89.98, -30),
    p2(773.40, -813.90, -303.11, -89.98, 30);

  cv::Mat base_to_ref, base_to_p1, base_to_p2, ref_to_p1, ref_to_p2,
    p1_to_ref, p2_to_ref;

  base_to_ref = ConvertRobotPose(ref);
  base_to_p1 = ConvertRobotPose(p1);
  base_to_p2 = ConvertRobotPose(p2);
  ref_to_p1 = base_to_p1 * base_to_ref.inv();
  ref_to_p2 = base_to_p2 * base_to_ref.inv();
  p1_to_ref = base_to_ref * base_to_p1.inv();
  p2_to_ref = base_to_ref * base_to_p2.inv();
  //std::cout << base_to_ref << std::endl;
  //std::cout << base_to_p1 << std::endl;
  //std::cout << base_to_p2 << std::endl;
  //std::cout << ref_to_p1 << std::endl;
  //std::cout << ref_to_p2 << std::endl;
  std::cout << p1_to_ref << std::endl;
  std::cout << p2_to_ref << std::endl;
  cv::FileStorage fs("test.xml", cv::FileStorage::WRITE);
  //fs << "image_corners" << image_corners;
  fs << "base_to_ref" << base_to_ref;
  fs << "base_to_p1" << base_to_p1;
  fs << "base_to_p2" << base_to_p2;
  fs << "ref_to_p1" << ref_to_p1;
  fs << "ref_to_p2" << ref_to_p2;
  fs.release();
  if (calib_pose_.size() < 3) {
    // error and return
  }
  for (int i = 0; i < calib_pose_.size(); ++i) {
    for (int j = i + 1; j < calib_pose_.size(); ++j) {
      for (int k = j + 1; k < calib_pose_.size(); ++k) {
        camera_ij = calib_pose_[j].world_to_camera_ *
          calib_pose_[i].world_to_camera_.inv();

        robot_ij = calib_pose_[j].base_to_robot_ *
          calib_pose_[i].base_to_robot_.inv();

        camera_jk = calib_pose_[k].world_to_camera_ *
          calib_pose_[j].world_to_camera_.inv();

        robot_jk = calib_pose_[k].base_to_robot_ *
          calib_pose_[j].base_to_robot_.inv();

        X = SolveX(camera_ij, robot_ij, camera_jk, robot_jk);
        error_frobenius = Frobenius(camera_ij * X - X * robot_ij) +
          Frobenius(camera_jk * X - X * robot_jk);
      }
    }
  }
  return 0.0;
}

cv::Mat HandEyeCalibrator::ConvertRobotPose(RobotPose robot) {
  cv::Mat base_to_robot, rmat_z, rmat_y, tvec;
  cv::Vec<double,3> t(robot.x_, robot.y_, robot.z_);
  rmat_z = cv::Mat_<double>::eye(3, 3);
  rmat_z.at<double>(0, 0) = cos(robot.rotate_angle_ * PI / 180);
  rmat_z.at<double>(1, 1) = cos(robot.rotate_angle_ * PI / 180);
  rmat_z.at<double>(0, 1) = sin(robot.rotate_angle_ * PI / 180);
  rmat_z.at<double>(1, 0) = -sin(robot.rotate_angle_ * PI / 180);

  rmat_y = cv::Mat_<double>::eye(3, 3);
  rmat_y.at<double>(0, 0) = cos(-robot.sway_angle_ * PI / 180);
  rmat_y.at<double>(2, 2) = cos(-robot.sway_angle_ * PI / 180);
  rmat_y.at<double>(0, 2) = -sin(-robot.sway_angle_ * PI / 180);
  rmat_y.at<double>(2, 0) = sin(-robot.sway_angle_ * PI / 180);

  tvec = -rmat_y * rmat_z * cv::Mat(t).reshape(1, 3);
  base_to_robot = cv::Mat_<double>::eye(4, 4);
  base_to_robot.rowRange(0, 3).colRange(0, 3) = rmat_y * rmat_z;
  tvec.copyTo(base_to_robot.rowRange(0, 3).col(3));
  return base_to_robot;
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
  // A = | RA   tA |
  //     |  0   1  |
  A1.rowRange(0, 3).colRange(0, 3).copyTo(RA1);
  A1.rowRange(0, 3).col(3).copyTo(tA1);
  B1.rowRange(0, 3).colRange(0, 3).copyTo(RB1);
  B1.rowRange(0, 3).col(3).copyTo(tB1);
  A2.rowRange(0, 3).colRange(0, 3).copyTo(RA2);
  A2.rowRange(0, 3).col(3).copyTo(tA2);
  B2.rowRange(0, 3).colRange(0, 3).copyTo(RB2);
  B2.rowRange(0, 3).col(3).copyTo(tB2);
  // solve RA * Rx = Rx * RB
  // by[eye(9) - kron(RA, RB)] * vec(Rx) = 0
  // the singular vector of the singular value 0 is v
  // R = alpha * reshape(v)
  C.rowRange(0, 9) = cv::Mat_<double>::eye(9, 9) - Kron(RA1, RB1);
  C.rowRange(9, 18) = cv::Mat_<double>::eye(9, 9) - Kron(RA2, RB2);
  cv::SVD::compute(C, w, u, vt);
  v = -vt.row(8).t();
  // solve RA * tx + tA = Rx * tB + tX
  //[eye(3) - RA]tx + alpha * kron[eye(3), tB']*v = tA
  //    N(3 * 3) *  tx + alpha * M(3 * 1) = tA
  //      [M  N] | alpha | = tA
  //             |  tx   |
  // solve this superlattice dislocation using least square method
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

  R = v.reshape(1, 3) * tx.at<double>(0, 0);
  cv::Mat rvec;
  cv::Rodrigues(R, rvec);
  cv::Rodrigues(rvec, R);
  X = cv::Mat_<double>::zeros(4, 4);
  R.copyTo(X.rowRange(0, 3).colRange(0, 3));
  tx.rowRange(1, 4).copyTo(X.rowRange(0, 3).col(3));
  X.at<double>(3, 3) = 1;
  return X;
}

double HandEyeCalibrator::Frobenius(cv::Mat A) {
  if (A.size == 0) {
    return 0;
  }
  double frobenius;
  cv::Mat square_A;
  square_A = A.mul(A);
  frobenius = sqrt(cv::sum(square_A)[0]);
  return frobenius;
}
