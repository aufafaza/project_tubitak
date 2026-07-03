#include "Detect.hpp" 
#include <opencv2/opencv.hpp>
#include <vector>
using namespace std;
using namespace cv;


Detect::Detect() : rect(false)  {} 

Detect::Detect(cv::VideoCapture v)  : v(v), rect(false){}; 

Detect::~Detect() = default; 

cv::Mat Detect::colorDetect(cv::Mat bgr){ 
	cv::Mat hsv; 
	
	cv::cvtColor(bgr, hsv, cv::COLOR_BGR2HSV);

	Mat red_lower, red_upper, red_mask; 
	Mat kernel = getStructuringElement(MORPH_ELLIPSE, Size(15, 15));
	Mat mask, mask1, mask2; 
	cv::inRange(hsv, cv::Scalar(0, 100, 80), cv::Scalar(15,  255, 255), mask1);
	cv::inRange(hsv, cv::Scalar(165, 100, 80), cv::Scalar(180, 255, 255), mask2);
	cv::bitwise_or(mask1, mask2, mask);
	cv::morphologyEx(mask, mask, MORPH_CLOSE, kernel, Point(-1, -1), 3);  
	erode(mask, mask, kernel, Point(-1, -1), 2);
	dilate(mask, mask, kernel, Point(-1, -1), 2);

	return mask; 
} 


cv::Mat Detect::colorDetectBlue(cv::Mat bgr){ 
	cv::Mat hsv, blur, mask; 
	GaussianBlur(bgr, blur, Size(7, 7), 0); 
	cv::cvtColor(bgr, hsv, cv::COLOR_BGR2HSV); 
	

	Mat kernel = getStructuringElement(MORPH_ELLIPSE, Size(15, 15));
	cv::inRange(hsv, cv::Scalar(100, 120, 50), cv::Scalar(130,255,255), mask);
	// dilate(mask, mask, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));	
	cv::morphologyEx(mask, mask, MORPH_CLOSE, kernel, Point(-1, -1), 3);  
	erode(mask, mask, kernel, Point(-1, -1), 2);  
	dilate(mask, mask, kernel, Point(-1, -1), 2); 
	return mask; 
}

int Detect::shapeDetect(Mat mask, Mat originalFrame){ 
	int edges;
	vector<vector<Point>> contours; 
	vector<cv::Vec4i> hierarchy;
	double perimeter, epsilon;
	vector<Point> approx;
	// TODO: optimize this contouring part
	// because now we use CHAIN_APPROX_SIMPLE, which uses 4 approximating points, it might be not too reliable
	// for the sake of memory cost 
	findContours(mask, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE); 
	for (size_t i = 0; i < contours.size(); i++){
		// Scalar color = Scalar( (0, 256), rng.uniform(0, 256), rng.uniform(0, 256));
		double area = contourArea(contours[i]);
		if (area < 500) continue;

		// TODO: tune this parameter
		perimeter = arcLength(contours[i], true); 
		epsilon = 0.03 * perimeter;

		approxPolyDP(contours[i], approx, epsilon, true); 

		edges = approx.size(); 
		string label; 

		switch (edges){
			case 4: 
				label = "RECTANGLE with AREA: " + to_string(area) + " found edges: " + to_string(edges); 
				rect = true; 
				break; 
			default: 
				label = "NOT DESIRED SHAPE with AREA: " + to_string(area) + " found edges: " + to_string(edges); 
				break; 
		}
		// TODO: use bounding boxes for visualization sake 
		// maybe also modularize this because this function is kind of bloated
		drawContours( originalFrame, contours, (int) i, Scalar(0, 255, 0), 2, LINE_8, hierarchy, 0);
		putText(originalFrame, label, approx[0], FONT_HERSHEY_SIMPLEX, 0.8, Scalar(0, 255, 0), 2); 
	}
	return edges; 
}