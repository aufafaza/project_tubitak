#include "uav_vision/segmentation.hpp"
#include <opencv2/opencv.hpp>
#include <vector>

Detect::Detect() : rect(false) {}

Detect::Detect(cv::VideoCapture v) : v(v), rect(false) {}

bool Detect::isValid() const { return v.isOpened(); }

std::optional<cv::Point2d> Detect::detect(cv::Mat& frame)
{
    rect = false;
    cv::Mat mask;
    cv::bitwise_or(redMask(frame), blueMask(frame), mask);
    auto centroids = findShapes(mask, frame);
    if (centroids.empty()) return std::nullopt;
    return centroids.front();
}

cv::Mat Detect::redMask(const cv::Mat& bgr)
{
    cv::Mat hsv, mask1, mask2, mask;
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(15, 15));

    cv::cvtColor(bgr, hsv, cv::COLOR_BGR2HSV);
    cv::inRange(hsv, cv::Scalar(0, 100, 80),   cv::Scalar(15,  255, 255), mask1);
    cv::inRange(hsv, cv::Scalar(165, 100, 80), cv::Scalar(180, 255, 255), mask2);
    cv::bitwise_or(mask1, mask2, mask);
    cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel, cv::Point(-1, -1), 3);
    cv::erode(mask,  mask, kernel, cv::Point(-1, -1), 2);
    cv::dilate(mask, mask, kernel, cv::Point(-1, -1), 2);
    return mask;
}

cv::Mat Detect::blueMask(const cv::Mat& bgr)
{
    cv::Mat hsv, blur, mask;
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(15, 15));

    cv::GaussianBlur(bgr, blur, cv::Size(7, 7), 0);
    cv::cvtColor(blur, hsv, cv::COLOR_BGR2HSV);
    cv::inRange(hsv, cv::Scalar(100, 120, 50), cv::Scalar(130, 255, 255), mask);
    cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel, cv::Point(-1, -1), 3);
    cv::erode(mask,  mask, kernel, cv::Point(-1, -1), 2);
    cv::dilate(mask, mask, kernel, cv::Point(-1, -1), 2);
    return mask;
}

std::vector<cv::Point2d> Detect::findShapes(const cv::Mat& mask, cv::Mat& frame)
{
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    findContours(mask, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
    std::vector<cv::Point2d> centroids; 

    for (size_t i = 0; i < contours.size(); i++) {
        if (cv::contourArea(contours[i]) < 500) continue;
        double perimeter = cv::arcLength(contours[i], true);
        std::vector<cv::Point> approx;
        cv::approxPolyDP(contours[i], approx, 0.03 * perimeter, true);

        int edges = static_cast<int>(approx.size());
        std::string label;

        if (edges == 4) {
            label = "RECTANGLE edges:" + std::to_string(edges);
            rect = true;
        } else {
            label = "UNKNOWN edges:" + std::to_string(edges);
        }

        cv::Moments m = cv::moments(contours[i]);
        if (m.m00 != 0) {
            cv::Point2d centroidPixel(m.m10 / m.m00, m.m01 / m.m00);
            centroids.push_back(centroidPixel);
            cv::circle(frame, centroidPixel, 5, cv::Scalar{0, 255, 0}, -1);
        }
        cv::drawContours(frame, contours, static_cast<int>(i),
                         cv::Scalar(0, 255, 0), 2, cv::LINE_8, hierarchy, 0);
        cv::putText(frame, label, approx[0],
                    cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 0), 2);
    }

    return centroids;
}
