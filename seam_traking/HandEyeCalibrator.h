#ifndef _HANDEYECALIBRATOR_H__
#define _HANDEYECALIBRATOR_H__

#include <opencv2/opencv.hpp>
#include <vector>
#include "CameraCalibrator.h"

class Camera;

class HandEyeCalibrator {
public:
  HandEyeCalibrator(Camera* cam, cv::Size board_size, double square_size);
  int AddPosePair(std::vector<cv::Mat>& src);
  bool ConvertRobotPose();
  void Calibrate();
  cv::Mat GetHandEyeTrans();
  
private:
  Camera* cam_;
  cv::Size board_size_;
  double square_size_;
  std::vector<cv::Point3f> object_points_;
  //void FindChessboardPose(cv::Mat& src, cv::Mat& pose, 
  //    bool is_board_reverse = false);

};
#endif /*_HANDEYECALIBRATOR_H__*/ 

