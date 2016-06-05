#include "HandEyeCalibrator.h"
#include <math.h>
#include "Matrix.h"
#include "Camera.h"
#include <fstream>

#define PI 3.14159265

HandEyeCalibrator::HandEyeCalibrator(Camera* cam, cv::Size board_size, 
  double square_size) :cam_(cam), board_size_(board_size), 
  square_size_(square_size) {}

std::vector<int> HandEyeCalibrator::AddPosePair(std::vector<cv::Mat>& src, 
    std::vector<RobotPose> robot_input, bool is_board_reverse) {

  CV_Assert(src.size() == robot_input.size());
  PosePair pose_pair;
  std::vector<int> error_pose_pair;
  calib_pose_.clear();
  for (unsigned int i = 0; i < src.size(); ++i) {
    pose_pair.base_to_robot_ = ConvertRobotPose(robot_input[i]);
    pose_pair.world_to_camera_ = cam_->FindChessboardPose(src[i], 
        board_size_, square_size_, is_board_reverse);

    cam_->camera_to_robot_ = pose_pair.base_to_robot_ * base_to_world_.inv() * 
      pose_pair.world_to_camera_.inv();

    std::cout << pose_pair.base_to_robot_ << std::endl;
    std::cout << pose_pair.world_to_camera_ << std::endl;
    std::cout << base_to_world_ << std::endl;
    std::cout << cam_->camera_to_robot_ << std::endl;

    if (abs(pose_pair.world_to_camera_.at<double>(3, 3) - 1) < 0.00001 &&
        abs(pose_pair.base_to_robot_.at<double>(3, 3) - 1) < 0.00001) {

      calib_pose_.push_back(pose_pair);
    } else {
      error_pose_pair.push_back(i + 1);
    }
  }
  return error_pose_pair;
}

void HandEyeCalibrator::GenerateBaseToWorld(std::vector<RobotPose> robot_input, 
    std::vector<cv::Point3d> world_points) {

  std::vector<cv::Mat> base_points;
  cv::Mat base_point, diff, world_point, rMat, tvec;
  double rotate_angle_base, rotate_angle_world, x, y;
  double x_sum, y_sum, z_sum, rotate_sum, rotate_ave;
  for (int i = 0; i < robot_input.size(); ++i) {
    base_point = cv::Mat_<double>::ones(4, 1);
    base_point.at<double>(0, 0) = robot_input[i].x_;
    base_point.at<double>(1, 0) = robot_input[i].y_;
    base_point.at<double>(2, 0) = robot_input[i].z_;
    base_point = Matrix::RotateX(180, 4) * base_point;
    base_points.push_back(base_point.clone());
  }

  rotate_sum = 0;
  for (int i = 1; i < base_points.size(); i++) {
    x = base_points[i].at<double>(0, 0) - base_points[0].at<double>(0, 0);
    y = base_points[i].at<double>(1, 0) - base_points[0].at<double>(1, 0);
    rotate_angle_base = std::acos(x / std::sqrt(x * x + y * y));
    if (y < 0) {
      rotate_angle_base = 2 * PI - rotate_angle_base;
    }
    x = world_points[i].x - world_points[0].x;
    y = world_points[i].y - world_points[0].y;
    rotate_angle_world = std::acos(x / std::sqrt(x * x + y * y));
    if (y < 0) {
      rotate_angle_world = 2 * PI - rotate_angle_world;
    }
    rotate_sum += rotate_angle_base - rotate_angle_world;
  }
  rotate_ave = rotate_sum / (base_points.size() - 1);
  rMat = Matrix::RotateZ(rotate_ave, 3, false) * Matrix::RotateX(180) ;

  tvec = cv::Mat_<double>::zeros(3, 1);
  for (int i = 0; i < world_points.size(); ++i) {
    world_point = Matrix::RotateZ(-rotate_ave, 3, false) * 
        cv::Mat(world_points[i]).reshape(1, 3);

    diff = base_points[i].rowRange(0,3) - world_point;
    tvec = tvec + diff;
    std::cout << diff << std::endl;
    std::cout << base_point << std::endl << std::endl;
  }
  tvec = tvec / world_points.size();
  tvec = Matrix::RotateX(-180, 3) * tvec;

  base_to_world_ = cv::Mat_<double>::eye(4, 4);
  rMat.copyTo(base_to_world_.rowRange(0, 3).colRange(0, 3));
  tvec = -rMat * tvec;
  tvec.copyTo(base_to_world_.rowRange(0, 3).col(3));
  std::cout << base_to_world_ << std::endl;

  for (int i = 0; i < base_points.size(); i++) {
    std::cout << base_to_world_ * Matrix::RotateX(-180,4) * base_points[i] 
        << std::endl;

  }
}

double HandEyeCalibrator::Calibrate() {
  //RobotPose p0(773.40, -638.90, -562.08, -89.98, 0),
  //  p1(481.51, -638.90, -562.08, -89.98, 0),
  //  p2(480.93, -880.07, -562.08, -89.98, 0),
  //  p3(773.40, -638.90, -410.86, -89.98, 0),
  //  p4(773.40, -537.40, -383.45, -89.98, -30);

  //cv::Mat pose0, pose1, pose2, pose3, pose4;
  //pose0 = ConvertRobotPose(p0);
  //pose1 = ConvertRobotPose(p1);
  //pose2 = ConvertRobotPose(p2);
  //pose3 = ConvertRobotPose(p3);
  //pose4 = ConvertRobotPose(p4);

  //std::cout << pose0 * pose1.inv() << std::endl;
  //std::cout << pose1 * pose2.inv() << std::endl;
  //std::cout << pose2 * pose3.inv() << std::endl;
  //std::cout << pose3 * pose4.inv() << std::endl;

  CV_Assert(calib_pose_.size() >= 3);
  cv::Mat camera_ij, camera_jk, robot_ij, robot_jk, X, minX;
  double error_frobenius, min_error;
  std::ofstream log;
  log.open("out.txt");
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

        log << i << " " << j << " " << k << std::endl;
        log << camera_ij << std::endl;
        log << camera_jk << std::endl;
        log << robot_ij << std::endl;
        log << robot_jk << std::endl;
        log << X << std::endl;
        log << error_frobenius << std::endl << std::endl;
        if (error_frobenius < min_error) {
          min_error = error_frobenius;
          cam_->camera_to_robot_ = X.inv();
        }
      }
    }
  }
  log.close();
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
