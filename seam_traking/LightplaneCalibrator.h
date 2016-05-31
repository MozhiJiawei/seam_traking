#ifndef _LIGHTPLANECALIBRATOR_H__
#define _LIGHTPLANECALIBRATOR_H__
#include <opencv2/opencv.hpp>
#include <vector>

class Camera;

class LightplaneCalibrator {
public:
  LightplaneCalibrator(Camera* cam, cv::Size board_size, double square_size);
  /* src_ref: Input chessboard images.
   is_board_reverse: Set true if the chessboard is surrounded by black pixels
   Return a vector showing the error chessboard image number.
   If there is no error, return an empty vector.  */
  std::vector<int> AddRefPose(std::vector<cv::Mat>& src_ref,
      bool is_board_reverse = false);

  /* src_light: Input light images
   Make sure that the light images is in the same order as the coressbounding
   chessboard images.
   Return a vector showing the error light image number.
   If there is no error, return an empty vector.  */
  std::vector<int> AddLightImage(std::vector<cv::Mat>& src_light);
  double Calibrate();
private:
  Camera* cam_;
  cv::Size board_size_;
  double square_size_; // unit: mm
  std::vector<cv::Mat> ref_pose_;
  std::vector<std::vector<cv::Point2f> > image_points_;
  std::vector<cv::Mat> light_point_cloud_;

  void GeneratePointCloud();
  cv::Mat Image2World(cv::Point2f& img_point, int ref_number);
  // Covert the input point in Referance n to Referance m
  cv::Mat Ref_n2Ref_m(int n, int m, cv::Mat& point_in_n);
};
#endif /*_LIGHTPLANECALIBRATOR_H__*/
