#include <opencv2/calib3d.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream> 

// change dimensions 
int CHECKERBOARD[2]{6,9}; 

int main () { 
	std::vector<std::vector<cv::Point3f>>  objpoints; 
	std::vector<std::vector<cv::Point2f>> imgpoints; 
	std::vector<cv::Point3f> objp; 

	for (int i = 0; i < CHECKERBOARD[1]; i++){ 
		for (int j = 0; j < CHECKERBOARD[0]; i++){ 
			objp.push_back(cv::Point3f(j, i, 0)); 
		}
	} 

	std::vector<cv::String> images; 

	std::string path = "to_add"; 
	
	// images to binary 
	cv::glob(path, images); 

	cv::Mat frame, gray; 

	std::vector<cv::Point2f> corner_pts;
	bool success; 

	for (int i = 0; i < images.size(); i++){ 
		frame = cv::imread(images[i]); 
		cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY); 

		success = cv::findChessboardCorners(gray, cv::Size(CHECKERBOARD[0], CHECKERBOARD[1]), corner_pts, cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_FAST_CHECK | cv::CALIB_CB_NORMALIZE_IMAGE);
		
		if (success) { 
			cv::TermCriteria criteria(cv::TermCriteria::EPS | cv::TermCriteria::MAX_ITER, 30, 0.001); 	

			cv::cornerSubPix(gray, corner_pts, cv::Size(11,11), cv::Size(-1, -1), criteria); 

			cv::drawChessboardCorners(frame, cv::Size(CHECKERBOARD[0], CHECKERBOARD[1]), corner_pts, success); 

			objpoints.push_back(objp); 
			imgpoints.push_back(corner_pts);
		}
		cv::imshow("Image", frame); 
		cv::waitKey(0); 
	} 
	
	cv::destroyAllWindows(); 
	cv::Mat cameraMatrix, distCoeffs, R, T; 

	cv::calibrateCamera(objpoints, imgpoints, cv::Size(gray.cols,gray.rows), cameraMatrix, distCoeffs, R, T);


	std::cout << "cameraMatrix : " << cameraMatrix << std::endl;
	std::cout << "distCoeffs : " << distCoeffs << std::endl;
	std::cout << "Rotation vector : " << R << std::endl;
	std::cout << "Translation vector : " << T << std::endl;
	cv::Mat image; 
	image = cv::imread(images[0]);
	cv::Mat dst, map1, map2, new_camera_matrix; 
	cv::Size imageSize(cv::Size(image.cols, image.rows)); 

	new_camera_matrix = cv::getOptimalNewCameraMatrix(cameraMatrix, distCoeffs, imageSize, 1, imageSize, 0);

	cv::undistort(frame, dst, new_camera_matrix, distCoeffs, new_camera_matrix);

	cv::initUndistortRectifyMap(cameraMatrix, distCoeffs, cv::Mat(), cv::getOptimalNewCameraMatrix(cameraMatrix, distCoeffs, imageSize, 1, imageSize, 0), imageSize, CV_16SC2, map1, map2);

	cv::remap(frame, dst, map1, map2, cv::INTER_LINEAR); 

	cv::imshow("undistorted image", dst); 
	cv::waitKey(0); 

	return 0;
} 
