#include "HandEyeCalibrator.h"
#include <math.h>
#include "Matrix.h"
#include "Camera.h"

#define PI 3.14159265

HandEyeCalibrator::HandEyeCalibrator(Camera* cam, cv::Size board_size, 
  double square_size) :cam_(cam), board_size_(board_size), 
  square_size_(square_size) {}

std::vector<int> HandEyeCalibrator::AddPosePair(std::vector<cv::Mat>& src, 
    std::vector<RobotPose> robot_input, bool is_board_reverse) {

  CV_Assert(src.size() == robot_input.size());
  PosePair pose_pair;
  std::vector<int> error_pose_pair;
  for (unsigned int i = 0; i < src.size(); ++i) {
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
  CV_Assert(calib_pose_.size() >= 3);
  cv::Mat camera_ij, camera_jk, robot_ij, robot_jk, X, minX;
  double error_frobenius, min_error;
  min_error = 1000;
  for (unsigned int i = 0; i < calib_pose_.size(); ++i) {
    for (unsigned int j = i + 1; j < calib_pose_.size(); ++j) {
      for (unsigned int k = j + 1; k < calib_pose_.size(); ++k) {
        camera_ij = calib_pose_[j].world_to_camera_ *
           calib_pose_[i].world_to_camera_.inv();

        robot_ij = calib_pose_[j].base_to_robot_ *
           calib_pose_[i].base_to_robot_.inv();

        camera_jk = calib_pose_[k].world_to_camera_ *
           calib_pose_[j].world_to_camera_.inv();

        robot_jk = calib_pose_[k].base_to_robot_ *
           calib_pose_[j].base_to_robot_.inv();

        X = SolveX(camera_ij, robot_ij, camera_jk, robot_jk);
        error_frobenius = Matrix::Frobenius(camera_ij * X - X * robot_ij) +
            Matrix::Frobenius(camera_jk * X - X * robot_jk);

        if (error_frobenius < min_error) {
          min_error = error_frobenius;
          cam_->robot_to_camera_ = X;
        }
      }
    }
  }
  return min_error;
}

cv::Mat HandEyeCalibrator::ConvertRobotPose(RobotPose robot) {
  cv::Mat base_to_robot, tvec, rmat;
  cv::Vec<double,3> t(robot.x_, robot.y_, robot.z_);
  rmat = Matrix::RotateX(-robot.sway_angle_) * 
      Matrix::RotateZ(robot.rotate_angle_) * Matrix::RotateZ(90);

  tvec = -rmat * cv::Mat(t).reshape(1, 3);
  base_to_robot = cv::Mat_<double>::eye(4, 4);
  rmat.copyTo(base_to_robot.rowRange(0, 3).colRange(0, 3));
  tvec.copyTo(base_to_robot.rowRange(0, 3).col(3));
  return base_to_robot;
 };

cv::Mat HandEyeCalibrator::SolveX(cv::Mat A1, cv::Mat B1,
    cv::Mat A2, cv::Mat B2) {

  CV_Assert(A1.rows == 4 && A1.cols == 4);
  CV_Assert(A2.rows == 4 && A2.cols == 4);
  CV_Assert(B1.rows == 4 && B1.cols == 4);
  CV_Assert(B2.rows == 4 && B2.cols == 4);
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
  C.rowRange(0, 9) = cv::Mat_<double>::eye(9, 9) - Matrix::Kron(RA1, RB1);
  C.rowRange(9, 18) = cv::Mat_<double>::eye(9, 9) - Matrix::Kron(RA2, RB2);
  cv::SVD::compute(C, w, u, vt);
  v = -vt.row(8).t();
  // solve RA * tx + tA = Rx * tB + tX
  //[eye(3) - RA]tx + alpha * kron[eye(3), tB']*v = tA
  //    N(3 * 3) *  tx + alpha * M(3 * 1) = tA
  //      [M  N] | alpha | = tA
  //             |  tx   |
  // solve this superlattice dislocation using least square method
  M1 = Matrix::Kron(cv::Mat_<double>::eye(3, 3), tB1.t()) * v;
  M2 = Matrix::Kron(cv::Mat_<double>::eye(3, 3), tB2.t()) * v;
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
