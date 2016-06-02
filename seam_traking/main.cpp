#include "CameraCalibrator.h"
#include "HandEyeCalibrator.h"
#include "LightplaneCalibrator.h"
#include "Camera.h"
#include <assert.h>

void main()
{
	cv::Mat src;
	std::string src_path;

	static const int WIDTH = 6;
	static const int HEIGHT = 5;
	cv::Size board_size(WIDTH, HEIGHT);
  double square_size = 10;
  Camera cam;
  CameraCalibrator cal(&cam, board_size);
  HandEyeCalibrator handeye(&cam, board_size, square_size);
  LightplaneCalibrator lightplane(&cam, board_size, square_size);

	std::vector<double> intrinsic;
  std::vector<double> distortion;

	std::vector<cv::Mat> mats;

	char* ptr = new char;
	for(int i=1;i<=27;i++) {
		std::sprintf(ptr,"%d",i);
		src_path = "calibimages\\";
		src = cv::imread(src_path + ptr + ".bmp");
		mats.push_back(src);
	}
	cv::Size image_size(mats[0].cols, mats[0].rows);
	cal.AddChessboardPoints(mats);
	cal.Calibrate(image_size);
  mats.clear();
  for (int i = 1; i <= 3; i++) {
    std::sprintf(ptr, "%d", i);
    src_path = "lightplaneimages\\";
    src = cv::imread(src_path + ptr + ".bmp");
    mats.push_back(src);
  }
  lightplane.AddRefPose(mats);
  mats.clear();
  for (int i = 4; i <= 6; i++) {
    std::sprintf(ptr, "%d", i);
    src_path = "lightplaneimages\\";
    src = cv::imread(src_path + ptr + ".bmp");
    mats.push_back(src);
  }
  lightplane.AddLightImage(mats);
  lightplane.Calibrate(); 
  mats.clear();
  for (int i = 1; i <= 6; i++) {
    std::sprintf(ptr, "%d", i);
    src_path = "handeyeimages\\";
    src = cv::imread(src_path + ptr + ".bmp");
    mats.push_back(src);
  }
  std::vector<RobotPose> robot_input;
  robot_input.push_back(RobotPose(866.45, -580.21, -877.67, -77.74, 0));
  robot_input.push_back(RobotPose(855.77, -586.03, -884.47, -81.93, 0));
  robot_input.push_back(RobotPose(762.7, -586.03, -871.62, -0.02, -15.42));
  robot_input.push_back(RobotPose(754.47, -563.15, -871.62, 3.31, -15.42));
  robot_input.push_back(RobotPose(789.67, -698.25, -865.48, 96.25, -15.76));
  robot_input.push_back(RobotPose(754.8, -571.16, -865.48, 5.43, -15.76));
  handeye.AddPosePair(mats, robot_input);
  handeye.Calibrate();

	//intrinsic.clear();
	//distortion.clear();

	//CvMat camera_matrix = cal.GetCameraMatrix();
	//CvMat *intrinsic_matrix = &camera_matrix;

	//intrinsic = std::vector<double>(
 //   intrinsic_matrix->data.db,
 //   intrinsic_matrix->data.db + 9);

	//CvMat dis_coeffs = cal.GetDisCoeffs();
	//CvMat *distortion_coeffs = &dis_coeffs;

	//distortion = std::vector<double>(
 //   distortion_coeffs->data.db,
 //   distortion_coeffs->data.db + 4);
	//
	//std::fstream in(src_path + "intrinsic.txt",std::ios::app);
	//in << "======intrinsic===============" << std::endl;
	//for(int i=0;i<9;i++)
	//	in << intrinsic[i] << std::endl;

	//std::fstream dis(src_path + "distortion.txt",std::ios::app);
	//dis << "======distortion===============" << std::endl;
	//for(int i=0;i<4;i++)
	//	dis << distortion[i] << std::endl; 

}