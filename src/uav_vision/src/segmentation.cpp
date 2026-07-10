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
    // cv::bitwise_or(redMask(frame), mask, mask);
    mask = redMask(frame); 
    auto centroids = findShapes(mask, frame);
    if (centroids.empty()) return std::nullopt;
    return centroids.front();
}

cv::Mat Detect::redMask(const cv::Mat& bgr)
{
    cv::Mat hsv, mask1, mask2, mask;
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3));
    cv::cvtColor(bgr, hsv, cv::COLOR_BGR2HSV);
    cv::inRange(hsv, cv::Scalar(0, 100, 80),   cv::Scalar(15,  255, 255), mask1);
    cv::inRange(hsv, cv::Scalar(165, 100, 80), cv::Scalar(180, 255, 255), mask2);
    cv::bitwise_or(mask1, mask2, mask);
    cv::morphologyEx(mask, mask, cv::MORPH_OPEN,  kernel);
    cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel);
    return mask;
}

cv::Mat Detect::blueMask(const cv::Mat& bgr)
{
    cv::Mat hsv, blur, mask;
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3));
    cv::GaussianBlur(bgr, blur, cv::Size(7, 7), 0);
    cv::cvtColor(blur, hsv, cv::COLOR_BGR2HSV);
    cv::inRange(hsv, cv::Scalar(100, 120, 50), cv::Scalar(130, 255, 255), mask);
    cv::morphologyEx(mask, mask, cv::MORPH_OPEN,  kernel);
    cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel);
    return mask;
}

std::vector<cv::Point2d> Detect::findShapes(const cv::Mat& mask, cv::Mat& frame)
{
    std::vector<std::vector<cv::Point>> contours;
    findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    cv::Point2d best;
    double max_area = 0.0;
    bool found = false;

    for (const auto &contour : contours) {
        double area = cv::contourArea(contour);
        if (area < 50.0) continue;

        std::vector<cv::Point> hull;
        cv::convexHull(contour, hull);
        double hull_area = cv::contourArea(hull);
        if (hull_area == 0.0) continue;

        double solidity = area / hull_area;
        if (solidity < 0.85) continue;

        if (area > max_area) {
            cv::Moments m = cv::moments(contour);
            if (m.m00 != 0) {
                max_area = area;
                best = cv::Point2d(m.m10 / m.m00, m.m01 / m.m00);
                found = true;
                cv::drawContours(frame, std::vector<std::vector<cv::Point>>{contour}, -1, cv::Scalar(0, 255, 0), 1);
            }
        }
    }

    if (!found) return {};
    cv::circle(frame, best, 5, cv::Scalar{0, 255, 0}, -1);
    return {best};
}
