#include "CameraCalibrator.h"
#include "Camera.h"
#include <assert.h>
#include <iostream>
#include <string>

CameraCalibrator::CameraCalibrator(Camera* cam,
    cv::Size & board_size): cam_(cam), board_size_(board_size) {}

int CameraCalibrator::AddChessboardPoints(
    std::vector<cv::Mat> &src, bool is_board_reverse) {

  std::vector<cv::Point2f> image_corners;
  std::vector<cv::Point3f> object_corners;

  for (int i = 0; i < board_size_.height; i++) {
    for (int j = 0; j < board_size_.width; j++) {
      object_corners.push_back(cv::Point3f((float)i*10, (float)j*10, 0.0f));
    }
  }

  cv::Mat image;
  int successes = 0;

  for (size_t i = 0; i < src.size(); i++) {
    image = src[i];
    int flag_ = CV_CALIB_CB_ADAPTIVE_THRESH + CV_CALIB_CB_FILTER_QUADS +
        CV_CALIB_CB_NORMALIZE_IMAGE + CV_CALIB_CB_FAST_CHECK;

    if (image.channels() != 1) {
      cv::cvtColor(image, image, CV_BGR2GRAY);
    }
    if (is_board_reverse) {
      image = 255 - image;
    }
    bool found = findChessboardCorners(image, board_size_, image_corners, flag_);
    if (found) {
      std::cout << "coners found  " << i << std::endl;
      cv::cornerSubPix(image,image_corners,
          cv::Size(11, 11), cv::Size(-1, -1),
          cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));

    }

    //cv::drawChessboardCorners(image, board_size_, image_corners, found);
    //cv::FileStorage fs("test.xml", cv::FileStorage::WRITE);
    //fs << "image_corners" << image_corners;
    //fs.release();
    //cv::imshow("image",image);
    //cv::waitKey(0);

    if ((int)image_corners.size() == board_size_.area()) {
      AddPoints(image_corners, object_corners);
      successes++;
    }
  }

  return successes;
}

void CameraCalibrator::AddPoints(const std::vector<cv::Point2f> &image_corners,
    const std::vector<cv::Point3f> &object_corners) {

  image_points_.push_back(image_corners);
  object_points_.push_back(object_corners);
}

double CameraCalibrator::Calibrate(cv::Size &image_size) {
  std::vector<cv::Mat> rvecs, tvecs;
  cv::Mat rMat, tvec;
  double error;
  cam_->image_size_ = image_size;
  error = cv::calibrateCamera(object_points_, image_points_, image_size,
      cam_->camera_matrix_, cam_->dist_coeffs_, 
      rvecs, tvecs, CV_CALIB_FIX_K4 | CV_CALIB_FIX_K5);

  //cv::FileStorage fs("rvecs.xml", cv::FileStorage::WRITE);
  //for (int i = 0; i < rvecs.size(); i++) {
  //  std::sprintf(ptr, "%d", i);
  //  name = "rvecs";
  //  fs << name + ptr << rvecs[i];

  //  name = "rMats";
  //  cv::Rodrigues(rvecs[i], rMat);
  //  fs << name + ptr << rMat;

  //  name = "tvecs";
  //  fs << name + ptr << tvecs[i];

  //  name = "RT";
  //  tvec = tvecs[i];
  //  tvec = rMat * tvec;
  //  fs << name + ptr << tvec;
  //}

  return error;

}
