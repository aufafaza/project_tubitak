#pragma once
#include <opencv2/opencv.hpp>
#include <optional>

class Detect {
public:
    Detect();
    explicit Detect(cv::VideoCapture v);
    ~Detect() = default;

    // Annotates frame in-place. Returns centroid pixel of detected target, or nullopt.
    std::optional<cv::Point2d> detect(cv::Mat& frame);
    cv::Mat redMask(const cv::Mat& bgr);
    cv::Mat blueMask(const cv::Mat& bgr);
    bool isValid() const;

private:
    cv::VideoCapture v;
    bool rect;

    std::vector<cv::Point2d> findShapes(const cv::Mat& mask, cv::Mat& frame);
};
