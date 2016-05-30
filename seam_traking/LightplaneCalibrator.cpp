#include "LightplaneCalibrator.h"
#include "Camera.h"
#include <fstream>

LightplaneCalibrator::LightplaneCalibrator(Camera * cam, 
    cv::Size board_size, double square_size)
  :cam_(cam), board_size_(board_size), square_size_(square_size)
{
}

std::vector<int> LightplaneCalibrator::AddRefPose(
    std::vector<cv::Mat>& src_ref, bool is_board_reverse) {

  cv::Mat pose;
  std::vector<int> error_images;
  for (int i = 0; i < src_ref.size(); ++i) {
    pose = cam_->FindChessboardPose(src_ref[i], board_size_, square_size_);
    if (abs(pose.at<double>(3, 3) - 1) < 0.00001) {
      ref_pose_.push_back(pose);
    } else {
      error_images.push_back(i + 1);
    }
  }
  return error_images;
}

std::vector<int> LightplaneCalibrator::AddLightImage(
    std::vector<cv::Mat>& src_light)

{
  std::ifstream in;
  int u, v, num, last_num;
  std::vector<cv::Point2f> image_point;
  std::vector<int> return_value;
  in.open("point_cloud.txt");
  last_num = num = 1;
  while (in.good()) {
    in >> u >> v >> num;
    if (num == last_num) {
      image_point.push_back(cv::Point2f(u, v));
    }
    else {
      image_points_.push_back(image_point);
      image_point.clear();
      image_point.push_back(cv::Point2f(u, v));
      last_num = num;
    }
  }
  image_points_.push_back(image_point);
  return return_value;
}

double LightplaneCalibrator::Calibrate() {
  GeneratePointCloud();
  cv::Mat pcloud(light_point_cloud_.size(), 3, CV_64F);
  cv::Mat light_plane_wcs(1,4,CV_64F);
  double x_ave, y_ave, z_ave;
  for (int i = 0; i < light_point_cloud_.size(); i++) {
    pcloud.row(i) = light_point_cloud_[i].t();
  }

  x_ave = mean(pcloud.col(0))(0);
  y_ave = mean(pcloud.col(1))(0);
  z_ave = mean(pcloud.col(2))(0);
  pcloud.col(0) = pcloud.col(0) - x_ave;
  pcloud.col(1) = pcloud.col(1) - y_ave;
  pcloud.col(2) = pcloud.col(2) - z_ave;

  cv::Mat w, u, vt;
  cv::SVD::compute(pcloud, w, u, vt);
  vt.row(2).copyTo(light_plane_wcs.colRange(0, 3));
  light_plane_wcs.at<double>(0, 3) = -
    light_plane_wcs.at<double>(0, 0) * x_ave -
    light_plane_wcs.at<double>(0, 1) * y_ave -
    light_plane_wcs.at<double>(0, 2) * z_ave;

  cam_->light_plane_ = light_plane_wcs * ref_pose_[0].inv();
  return 0.0;
}

void LightplaneCalibrator::GeneratePointCloud()
{
  cv::Mat point_in_ref0, point_in_refi;
  for (int i = 0; i < image_points_.size(); i++) {
    for (int j = 0; j < image_points_[i].size(); j++) {
      point_in_refi = Image2World(image_points_[i][j], i);
      point_in_ref0 = Ref_n2Ref_m(i, 0, point_in_refi);
      light_point_cloud_.push_back(point_in_ref0.rowRange(0,3));
    }
  }
}

cv::Mat LightplaneCalibrator::Image2World(cv::Point2f & img_point,
    int ref_number) {
  
  cv::Mat plane_ccs, line_ccs;
  // Solve linear equation: A * result_ccs = B
  // result_wcs = inv(ref_pose) * result_ccs
  cv::Mat A, B, result_ccs, result_wcs;
  plane_ccs = cv::Mat_<double>::zeros(1, 4);
  plane_ccs.at<double>(0, 2) = 1;
  plane_ccs = plane_ccs * ref_pose_[ref_number].inv();

  line_ccs = cv::Mat_<double>::zeros(3, 4);
  cam_->camera_matrix_.copyTo(line_ccs.colRange(0, 3));
  line_ccs.at<double>(0, 2) -= img_point.x;
  line_ccs.at<double>(1, 2) -= img_point.y;

  A = cv::Mat_<double>::zeros(3, 3);
  plane_ccs.colRange(0, 3).copyTo(A.rowRange(0, 1));
  line_ccs.rowRange(0, 2).colRange(0, 3).copyTo(A.rowRange(1, 3));

  B = cv::Mat_<double>::zeros(3, 1);
  plane_ccs.col(3).copyTo(B.rowRange(0, 1));
  line_ccs.rowRange(0, 2).col(3).copyTo(B.rowRange(1, 3));
  B = B * -1;

  result_ccs = A.inv() * B;
  result_wcs = cv::Mat_<double>::ones(4, 1);
  result_ccs.copyTo(result_wcs.rowRange(0, 3));
  result_wcs = ref_pose_[ref_number].inv() * result_wcs;

  return result_wcs;
}

cv::Mat LightplaneCalibrator::Ref_n2Ref_m(int n, int m, cv::Mat& point_in_n) {

  if (n == m) {
    return point_in_n;
  }
  else {
    cv::Mat point_in_m;
    point_in_m = ref_pose_[m].inv() * ref_pose_[n] * point_in_n;
    return point_in_m;
  }
}
