#include "Camera.h"

Camera::Camera():must_init_distort_(true) {
  light_plane_ = cv::Mat_<double>::zeros(1, 4);
};

void Camera::UndistorImage(cv::Mat& src, cv::Mat& dst) { 
  if (must_init_distort_) {
    cv::initUndistortRectifyMap(camera_matrix_, dist_coeffs_, 
        cv::Mat_<double>::eye(3, 3), camera_matrix_, image_size_, 
        CV_16SC2, map1_, map2_);
    must_init_distort_ = false;

  }
  cv::remap(src, dst, map1_, map2_, CV_INTER_LINEAR);
}

cv::Mat Camera::FindChessboardPose(cv::Mat & src, cv::Size board_size, 
    double square_size, bool is_board_reverse) {

  cv::Mat img_undistort;
  std::vector<cv::Point3f> object_points;
  std::vector<cv::Point2f> img_points;
  cv::Mat rvec, tvec, rMat, world_to_camera;
  for (int y = 0; y < board_size.height; y++) {
    for (int x = 0; x < board_size.width; x++) {
      object_points.push_back(cv::Point3f(x * square_size,
        y * square_size, 0));

    }
  }
  UndistorImage(src, img_undistort);
  int flag_ = CV_CALIB_CB_ADAPTIVE_THRESH + CV_CALIB_CB_FILTER_QUADS +
    CV_CALIB_CB_NORMALIZE_IMAGE + CV_CALIB_CB_FAST_CHECK;

  if (img_undistort.channels() != 1) {
    cv::cvtColor(img_undistort, img_undistort, CV_BGR2GRAY);
  }
  if (is_board_reverse) {
    img_undistort = 255 - img_undistort;
  }
  bool found = findChessboardCorners(img_undistort, board_size,
    img_points, flag_);

  if (found) {
    cv::cornerSubPix(img_undistort, img_points,
      cv::Size(11, 11), cv::Size(-1, -1),
      cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));

  }
  else {
    return cv::Mat_<double>::zeros(4, 4);
  }
  cv::solvePnP(object_points, img_points, camera_matrix_,
    cv::Mat_<double>::zeros(1, 5), rvec, tvec);

  world_to_camera = cv::Mat_<double>::eye(4, 4);
  cv::Rodrigues(rvec, rMat);
  rMat.copyTo(world_to_camera.rowRange(0, 3).colRange(0, 3));
  tvec.copyTo(world_to_camera.rowRange(0, 3).col(3));
  return world_to_camera;
}
