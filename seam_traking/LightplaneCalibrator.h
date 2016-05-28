#ifndef _LIGHTPLANECALIBRATOR_H__
#define _LIGHTPLANECALIBRATOR_H__
#include <opencv2/opencv.hpp>
#include <vector>

class Camera;

class LightplaneCalibrator {
public:
  LightplaneCalibrator(Camera* cam, cv::Size board_size, double square_size);
  int AddRefPose(std::vector<cv::Mat>& src_ref, bool is_board_reverse = false);
  int AddLightImage(std::vector<cv::Mat>& src_light);
  double Calibrate();
private:
  Camera* cam_;
  cv::Size board_size_;
  double square_size_;
  std::vector<std::vector<cv::Point2f> > image_points_;
  std::vector<cv::Mat> light_point_cloud_;
  std::vector<cv::Mat> ref_pose_;

  void GeneratePointCloud();
  cv::Mat Image2World(cv::Point2f& img_point, int ref_number);
  // Covert the input point in Referance n to Referance m
  cv::Mat Ref_n2Ref_m(int n, int m, cv::Mat& point_in_n);
};
#endif /*_LIGHTPLANECALIBRATOR_H__*/