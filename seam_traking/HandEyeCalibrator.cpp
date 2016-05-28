#include "HandEyeCalibrator.h"
#include "Camera.h"

#define Test 1;

HandEyeCalibrator::HandEyeCalibrator(Camera* cam, cv::Size board_size, 
  double square_size) :cam_(cam), board_size_(board_size), 
  square_size_(square_size) {}

int HandEyeCalibrator::AddPosePair(std::vector<cv::Mat>& src) {
  cv::Mat pose;
  //for (int i = 0; i < src.size(); i++) {
  //  FindChessboardPose(src[i], pose);
  //}
  pose = cam_->FindChessboardPose(src[0], board_size_, square_size_);
  return 0;
}

bool HandEyeCalibrator::ConvertRobotPose() {
  return true;
}

void HandEyeCalibrator::Calibrate() {

}

cv::Mat HandEyeCalibrator::GetHandEyeTrans() {
  return cv::Mat();
}
