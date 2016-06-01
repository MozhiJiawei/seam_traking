#include "LightplaneCalibrator.h"
#include "Camera.h"
#include <fstream>

LightplaneCalibrator::LightplaneCalibrator(Camera * cam, 
    cv::Size board_size, double square_size)
  :cam_(cam), board_size_(board_size), square_size_(square_size)
{
}

std::vector<int> LightplaneCalibrator::AddRefPose(
    std::vector<cv::Mat>& src_ref, bool is_board_reverse) {

  cv::Mat pose;
  std::vector<int> error_images;
  for (int i = 0; i < src_ref.size(); ++i) {
    pose = cam_->FindChessboardPose(src_ref[i], board_size_, square_size_);
    if (abs(pose.at<double>(3, 3) - 1) < 0.00001) {
      ref_pose_.push_back(pose);
    } else {
      error_images.push_back(i + 1);
    }
  }
  return error_images;
}

std::vector<int> LightplaneCalibrator::AddLightImage(
    std::vector<cv::Mat>& srcs_light) {

  std::vector<int> return_value;
  //std::ifstream in;
  //int u, v, num, last_num;
  //std::vector<cv::Point2f> image_point;
  //in.open("point_cloud.txt");
  //last_num = num = 1;
  //while (in.good()) {
  //  in >> u >> v >> num;
  //  if (num == last_num) {
  //    image_point.push_back(cv::Point2f(u, v));
  //  }
  //  else {
  //    image_points_.push_back(image_point);
  //    image_point.clear();
  //    image_point.push_back(cv::Point2f(u, v));
  //    last_num = num;
  //  }
  //}
  //image_points_.push_back(image_point);
	for ( int i=0; i < srcs_light.size(); i++ ) {
		std::vector<cv::Point> pts ; std::vector<cv::Point2f> ptfs;
		cv::Mat im = srcs_light[i], gray;
		if( im.channels() == 3 )
			cvtColor(im, gray, CV_BGR2GRAY);
		else
			gray =  im;
		medianBlur ( gray, gray, 11 );
		cv::Mat threshold_out;
		cv::threshold(gray, threshold_out, 25, 255, CV_THRESH_BINARY);
    cv::Mat dialateStructure = cv::getStructuringElement(
        cv::MorphShapes::MORPH_RECT , cv::Size(15, 15));

		dilate(threshold_out, threshold_out, dialateStructure, cv::Point(-1, -1));
#ifdef _DEBUG_JIANG_  // for debug
		namedWindow("threshold_medianBlur_out");
		imshow("threshold_medianBlur_out",threshold_out);
		/*imwrite("threshold_medianBlur_out.bmp",threshold_out);*/
		waitKey(200);
#endif
		std::vector<std::vector<cv::Point> > contours;
		std::vector<cv::Vec4i> hierarchy;
		findContours( threshold_out, contours, hierarchy, CV_RETR_EXTERNAL,
        CV_CHAIN_APPROX_NONE , cv::Point(0, 0) );

		/// Draw contours
		cv::Mat drawing = cv::Mat::zeros( threshold_out.size(), CV_8U );
		for( size_t i = 0; i< contours.size(); i++ ) {
			if (contours[i].size() > 200) {
				//Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
				drawContours( drawing, contours, (int)i, cv::Scalar(255),
            CV_FILLED , 8, hierarchy, 0, cv::Point() );

			}
		}
		cv::Mat  thining_output;
		Thinning(drawing, thining_output);
#ifdef _DEBUG_JIANG_
		cv::imshow("thining_output",thining_output);
		waitKey(200);
#endif
		findNonZero( thining_output, pts );
		ptfs.assign(pts.begin(),pts.end());
		ims_points_.push_back( ptfs );
	}
  return return_value;
}

double LightplaneCalibrator::Calibrate() {
  GeneratePointCloud();
  cv::Mat pcloud(light_point_cloud_.size(), 3, CV_64F);
  cv::Mat light_plane_wcs(1,4,CV_64F);
  double x_ave, y_ave, z_ave;
  for (int i = 0; i < light_point_cloud_.size(); i++) {
    pcloud.row(i) = light_point_cloud_[i].t();
  }

  x_ave = mean(pcloud.col(0))(0);
  y_ave = mean(pcloud.col(1))(0);
  z_ave = mean(pcloud.col(2))(0);
  pcloud.col(0) = pcloud.col(0) - x_ave;
  pcloud.col(1) = pcloud.col(1) - y_ave;
  pcloud.col(2) = pcloud.col(2) - z_ave;

  cv::Mat w, u, vt;
  cv::SVD::compute(pcloud, w, u, vt);
  vt.row(2).copyTo(light_plane_wcs.colRange(0, 3));
  light_plane_wcs.at<double>(0, 3) = -
    light_plane_wcs.at<double>(0, 0) * x_ave -
    light_plane_wcs.at<double>(0, 1) * y_ave -
    light_plane_wcs.at<double>(0, 2) * z_ave;

  cam_->light_plane_ = light_plane_wcs * ref_pose_[0].inv();
  return 0.0;
}

void LightplaneCalibrator::GeneratePointCloud() {
  cv::Mat point_in_ref0, point_in_refi;
  for (int i = 0; i < ims_points_.size(); i++) {
    for (int j = 0; j < ims_points_[i].size(); j++) {
      point_in_refi = Image2World(ims_points_[i][j], i);
      point_in_ref0 = Ref_n2Ref_m(i, 0, point_in_refi);
      light_point_cloud_.push_back(point_in_ref0.rowRange(0,3));
    }
  }
}

cv::Mat LightplaneCalibrator::Image2World(cv::Point2f & img_point,
    int ref_number) {
  
  cv::Mat plane_ccs, line_ccs;
  // Solve linear equation: A * result_ccs = B
  // result_wcs = inv(ref_pose) * result_ccs
  cv::Mat A, B, result_ccs, result_wcs;
  plane_ccs = cv::Mat_<double>::zeros(1, 4);
  plane_ccs.at<double>(0, 2) = 1;
  plane_ccs = plane_ccs * ref_pose_[ref_number].inv();

  line_ccs = cv::Mat_<double>::zeros(3, 4);
  cam_->camera_matrix_.copyTo(line_ccs.colRange(0, 3));
  line_ccs.at<double>(0, 2) -= img_point.x;
  line_ccs.at<double>(1, 2) -= img_point.y;

  A = cv::Mat_<double>::zeros(3, 3);
  plane_ccs.colRange(0, 3).copyTo(A.rowRange(0, 1));
  line_ccs.rowRange(0, 2).colRange(0, 3).copyTo(A.rowRange(1, 3));

  B = cv::Mat_<double>::zeros(3, 1);
  plane_ccs.col(3).copyTo(B.rowRange(0, 1));
  line_ccs.rowRange(0, 2).col(3).copyTo(B.rowRange(1, 3));
  B = B * -1;

  result_ccs = A.inv() * B;
  result_wcs = cv::Mat_<double>::ones(4, 1);
  result_ccs.copyTo(result_wcs.rowRange(0, 3));
  result_wcs = ref_pose_[ref_number].inv() * result_wcs;

  return result_wcs;
}

cv::Mat LightplaneCalibrator::Ref_n2Ref_m(int n, int m, cv::Mat& point_in_n) {

  if (n == m) {
    return point_in_n;
  }
  else {
    cv::Mat point_in_m;
    point_in_m = ref_pose_[m].inv() * ref_pose_[n] * point_in_n;
    return point_in_m;
  }
}

void LightplaneCalibrator::Thinning(const cv::Mat & src, cv::Mat & dst) {
	dst = src.clone();
	dst /= 255;         // convert to binary image

	cv::Mat prev = cv::Mat::zeros(dst.size(), CV_8UC1);
	cv::Mat diff;

	do {
		ThinningIteration(dst, 0);
		ThinningIteration(dst, 1);
		cv::absdiff(dst, prev, diff);
		dst.copyTo(prev);
	} 
	while (cv::countNonZero(diff) > 0);

	dst *= 255;

}

void LightplaneCalibrator::ThinningIteration(cv::Mat & img, int iter) {
	CV_Assert(img.channels() == 1);
	CV_Assert(img.depth() != sizeof(uchar));
	CV_Assert(img.rows > 3 && img.cols > 3);

	cv::Mat marker = cv::Mat::zeros(img.size(), CV_8UC1);

	int nRows = img.rows;
	int nCols = img.cols;

	if (img.isContinuous()) {
		nCols *= nRows;
		nRows = 1;
	}

	int x, y;
	uchar *pAbove;
	uchar *pCurr;
	uchar *pBelow;
	uchar *nw, *no, *ne;    // north (pAbove)
	uchar *we, *me, *ea;
	uchar *sw, *so, *se;    // south (pBelow)

	uchar *pDst;

	// initialize row pointers
	pAbove = NULL;
	pCurr  = img.ptr<uchar>(0);
	pBelow = img.ptr<uchar>(1);

	for (y = 1; y < img.rows-1; ++y) {
		// shift the rows up by one
		pAbove = pCurr;
		pCurr  = pBelow;
		pBelow = img.ptr<uchar>(y+1);

		pDst = marker.ptr<uchar>(y);

		// initialize col pointers
		no = &(pAbove[0]);
		ne = &(pAbove[1]);
		me = &(pCurr[0]);
		ea = &(pCurr[1]);
		so = &(pBelow[0]);
		se = &(pBelow[1]);

		for (x = 1; x < img.cols-1; ++x) {
			// shift col pointers left by one (scan left to right)
			nw = no;
			no = ne;
			ne = &(pAbove[x+1]);
			we = me;
			me = ea;
			ea = &(pCurr[x+1]);
			sw = so;
			so = se;
			se = &(pBelow[x+1]);

			int A  = (*no == 0 && *ne == 1) + (*ne == 0 && *ea == 1) + 
				(*ea == 0 && *se == 1) + (*se == 0 && *so == 1) + 
				(*so == 0 && *sw == 1) + (*sw == 0 && *we == 1) +
				(*we == 0 && *nw == 1) + (*nw == 0 && *no == 1);
			int B  = *no + *ne + *ea + *se + *so + *sw + *we + *nw;
			int m1 = iter == 0 ? (*no * *ea * *so) : (*no * *ea * *we);
			int m2 = iter == 0 ? (*ea * *so * *we) : (*no * *so * *we);

			if (A == 1 && (B >= 2 && B <= 6) && m1 == 0 && m2 == 0)
				pDst[x] = 1;
		}
	}
	img &= ~marker;
}
