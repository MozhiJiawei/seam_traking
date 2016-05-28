#ifndef _CAMERACALIBRATOR_H__
#define _CAMERACALIBRATOR_H__

#include <opencv2/opencv.hpp>
#include <vector>
#include <fstream>

class Camera;

class CameraCalibrator {
 public:
  CameraCalibrator(Camera* cam, cv::Size& board_sizes); 
  int AddChessboardPoints(std::vector<cv::Mat> &src, 
      bool is_board_reverse = false);

  void AddPoints(const std::vector<cv::Point2f> &image_corners,
      const std::vector<cv::Point3f> &object_corners);

  double Calibrate(cv::Size &image_size);

 private:
  Camera* cam_;
  cv::Size board_size_;
  int flag_; // calibrating mode
  std::vector<std::vector<cv::Point3f> >object_points_;
  std::vector<std::vector<cv::Point2f> >image_points_;

};
#endif /*_CAMERACALIBRATOR_H__*/
