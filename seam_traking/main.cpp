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
	for(int i=0;i<=28;i++) {
		std::sprintf(ptr,"%d",i);
		src_path = "calibimages\\";
		src = cv::imread(src_path + ptr + ".bmp");
		mats.push_back(src);
	}
	cv::Size image_size(mats[0].cols, mats[0].rows);
	cal.AddChessboardPoints(mats);
	cal.Calibrate(image_size);
  mats.clear();
  //for (int i = 1; i <= 8; i++) {
  //  std::sprintf(ptr, "%d", i);
  //  src_path = "distort\\";
  //  src = cv::imread(src_path + ptr + ".bmp");
  //  cam.UndistorImage(src, src);
  //  src_path = "undistort\\"; 
  //  cv::imwrite(src_path + ptr + ".bmp", src);
  //}
  //
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
 // for (int i = 1; i <= 6; i++) {
 //   std::sprintf(ptr, "%d", i);
 //   src_path = "handeyeimages\\";
 //   src = cv::imread(src_path + ptr + ".bmp");
 //   mats.push_back(src);
 // }
 // std::vector<RobotPose> robot_input;
 // //robot_input.push_back(RobotPose(773.40, -638.90, -562.08, -89.98, 0));
 // //robot_input.push_back(RobotPose());
 // //robot_input.push_back(RobotPose());
 // //robot_input.push_back(RobotPose());
 // //robot_input.push_back(RobotPose());
 // //robot_input.push_back(RobotPose());
 // robot_input.push_back(RobotPose(866.45, -580.21, -877.67, -77.74, 0));
 // robot_input.push_back(RobotPose(855.77, -586.03, -884.47, -81.93, 0));
 // robot_input.push_back(RobotPose(762.7, -586.03, -871.62, -0.02, -15.42));
 // robot_input.push_back(RobotPose(754.47, -563.15, -871.62, 3.31, -15.42));
 // robot_input.push_back(RobotPose(789.67, -698.25, -865.48, 96.25, -15.76));
 // robot_input.push_back(RobotPose(754.8, -571.16, -865.48, 5.43, -15.76));
 // handeye.AddPosePair(mats, robot_input);
  std::vector<RobotPose> robot_input;
  robot_input.push_back(RobotPose(787.45, -696.22, -664.02, -80.35, 0));
  robot_input.push_back(RobotPose(739.05, -709.50, -664.02, -80.35, 0));
  robot_input.push_back(RobotPose(728.80, -670.40, -664.02, -80.35, 0));
  robot_input.push_back(RobotPose(777.51, -657.28, -664.02, -80.35, 0));
  robot_input.push_back(RobotPose(770.30, -669.98, -664.02, -80.35, 0));
  robot_input.push_back(RobotPose(746.00, -697.03, -664.02, -80.59, 0));
  std::vector<cv::Point3d> world_points;
  world_points.push_back(cv::Point3d(0, 0, 0));
  world_points.push_back(cv::Point3d(50, 0, 0));
  world_points.push_back(cv::Point3d(50, 40, 0));
  world_points.push_back(cv::Point3d(0, 40, 0));
  world_points.push_back(cv::Point3d(10, 30, 0));
  world_points.push_back(cv::Point3d(40, 10, 0));
  handeye.GenerateBaseToWorld(robot_input, world_points);
  mats.clear();
  for (int i = 0; i <= 0; i++) {
    std::sprintf(ptr, "%d", i);
    src_path = "handeyeimages\\";
    src = cv::imread(src_path + ptr + ".bmp");
    mats.push_back(src);
  }
  robot_input.clear();
  robot_input.push_back(RobotPose(787.31, -664.76, -664.02, -80.59, 0));
  handeye.AddPosePair(mats, robot_input);
  cam.PixelToRobot(cv::Point2d(1105, 558));
  cam.PixelToRobot(cv::Point2d(1124, 426));
  cam.PixelToRobot(cv::Point2d(1119, 357));
  cam.PixelToRobot(cv::Point2d(1133, 284));

  return;
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