#ifndef _CAMERA_H__
#define _CAMERA_H__
#include <opencv2/opencv.hpp>
#include <vector>

class Camera {
public:
  Camera();
  friend class CameraCalibrator;
  friend class HandEyeCalibrator;
  friend class LightplaneCalibrator;

  void UndistorImage(cv::Mat& src, cv::Mat& dst);
  // Get pose: chessboard world --> camera
  cv::Mat FindChessboardPose(cv::Mat& src, cv::Size board_size,
    double square_size, bool is_board_reverse = false);

private:
  bool must_init_distort_;
  cv::Mat dist_coeffs_;
  cv::Mat map1_, map2_;
  // Set after camera calibration
  cv::Size image_size_;
  cv::Mat camera_matrix_;
  // Set after lightplane calibration
  cv::Mat light_plane_;
  // Set after handeye calibration
  cv::Mat robot_to_camera_;
};

#endif /*_CAMERA_H__*/