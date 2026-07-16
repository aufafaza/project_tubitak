#include "uav_vision/segmentation.hpp"
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <vector>

Detect::Detect() : rect(false) {}

Detect::Detect(cv::VideoCapture v) : v(v), rect(false) {}

bool Detect::isValid() const { return v.isOpened(); }

std::optional<cv::Point2d> Detect::detect(cv::Mat& frame)
{
    rect = false;
    cv::Mat mask;
    // cv::bitwise_or(redMask(frame), blueMask(frame), mask);
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
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    if (contours.empty())
        return mask;
 
    std::vector<cv::Point> allPoints;
    for (const auto& c : contours) {
        if (cv::contourArea(c) < 50.0) continue;
        allPoints.insert(allPoints.end(), c.begin(), c.end());
    }
    if (allPoints.empty())
        return mask;

    std::vector<cv::Point> hull;
    cv::convexHull(allPoints, hull);

    cv::Mat filled = cv::Mat::zeros(mask.size(), CV_8U);
    cv::drawContours(filled, std::vector<std::vector<cv::Point>>{hull}, -1, cv::Scalar(255), cv::FILLED);
    return filled;
}

cv::Mat Detect::blueMask(const cv::Mat& bgr)
{
    cv::Mat hsv, blur, mask;
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3));
    // cv::GaussianBlur(bgr, blur, cv::Size(7, 7), 0);
    cv::cvtColor(bgr, hsv, cv::COLOR_BGR2HSV);
    cv::inRange(hsv, cv::Scalar(100, 120, 50), cv::Scalar(130, 255, 255), mask);
    cv::morphologyEx(mask, mask, cv::MORPH_OPEN,  kernel);
    // cv::morphologyEx(mask, mask, cv::MORPH_TOPHAT,  kernel);
    cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel);
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    if (contours.empty())
        return mask;
 
    std::vector<cv::Point> allPoints;
    for (const auto& c : contours) {
        if (cv::contourArea(c) < 50.0) continue;
        allPoints.insert(allPoints.end(), c.begin(), c.end());
    }
    if (allPoints.empty())
        return mask;

    std::vector<cv::Point> hull;
    cv::convexHull(allPoints, hull);

    cv::Mat filled = cv::Mat::zeros(mask.size(), CV_8U);
    cv::drawContours(filled, std::vector<std::vector<cv::Point>>{hull}, -1, cv::Scalar(255), cv::FILLED);
    return filled;
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
        if (area < 600.0) continue;

        std::vector<cv::Point> hull;
        cv::convexHull(contour, hull);
        double hull_area = cv::contourArea(hull);
        if (hull_area < 1.0) continue;
        if (area / hull_area < 0.85) continue;

        std::vector<cv::Point> approx;
        cv::approxPolyDP(contour, approx, 0.105 * cv::arcLength(contour, true), true);
        if (approx.size() != 4) continue;
        if (!cv::isContourConvex(approx)) continue;

        cv::RotatedRect rr = cv::minAreaRect(approx);
        float ar = (rr.size.height > 0) ? rr.size.width / rr.size.height : 0;
        if (ar < 0.5f || ar > 2.0f) continue;

        if (area > max_area) {
            cv::Moments m = cv::moments(contour);
            if (m.m00 != 0) {
                max_area = area;
                best = cv::Point2d(m.m10 / m.m00, m.m01 / m.m00);
                found = true;
                cv::drawContours(frame, std::vector<std::vector<cv::Point>>{approx}, -1, cv::Scalar(0, 255, 0), 2);
            }
        }
    }

    if (!found) return {};
    cv::circle(frame, best, 5, cv::Scalar{0, 255, 0}, -1);
    return {best};
}

// int main()
// {
//     cv::VideoCapture cap("/home/fazabobi/project_tubitak/src/uav_vision/src/test_clip_7.mp4");
//
//     if (!cap.isOpened()) {
//         std::cerr << "Failed to open video file" << std::endl;
//         return 1;
//     }
//
//     Detect detector(cap);
//     cv::Mat frame;
//
//     while (true) {
//         cap >> frame;
//         if (frame.empty()) break; 
//         auto blue_mask = detector.blueMask(frame); 
//         auto red_mask = detector.redMask(frame);  
//         auto centroid = detector.detect(frame);
//
//         if (centroid.has_value()) {
//             std::cout << "Detected at: " << centroid->x << ", " << centroid->y << std::endl;
//         }
//
//         cv::imshow("Detection", frame);
//         cv::imshow("Blue Mask", blue_mask);
//         cv::imshow("Red Mask", red_mask);
//         // cv::namedWindow("Detection", cv::WINDOW_NORMAL);
//         // cv::namedWindow("Blue Mask", cv::WINDOW_NORMAL);
//         // cv::namedWindow("Red Mask", cv::WINDOW_NORMAL);
//         // cv::moveWindow("Detection", 0, 0);
//         // cv::moveWindow("Blue Mask", frame.cols, 0);
//         // cv::moveWindow("Red Mask", 0, frame.rows + 50);
//         int key = cv::waitKey(30) & 0xFF;
//         if (key == 27) break; 
//     }
//
//     cap.release();
//     cv::destroyAllWindows();
//     return 0;
// }
