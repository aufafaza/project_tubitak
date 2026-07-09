#include "drone_vision/vision_node.hpp"

TargetVisionProcessor::TargetVisionProcessor() : Node("target_vision_processor") {
    // ROS 2 Subscriptions
    camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
        "/camera/camera_info", 10,
        std::bind(&TargetVisionProcessor::cameraInfoCallback, this, std::placeholders::_1));

    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/camera/image", 10,
        std::bind(&TargetVisionProcessor::imageCallback, this, std::placeholders::_1));

    // ROS 2 Publishers
    red_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>("/vision/red_target_pixel", 10);
    blue_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>("/vision/blue_target_pixel", 10);
    debug_img_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/vision/debug_frame", 10);

    morph_kernel_ = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
    RCLCPP_INFO(this->get_logger(), "C++ TargetVisionProcessor Node Initialized.");
}

void TargetVisionProcessor::cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
    if (!camera_locked_) {
        // Build camera matrix K (3x3)
        K_ = cv::Mat(3, 3, CV_64F, (void*)msg->k.data()).clone();
        // Build distortion coefficients D (variable length)
        D_ = cv::Mat(msg->d.size(), 1, CV_64F, (void*)msg->d.data()).clone();

        int h = msg->height > 0 ? msg->height : 480;
        int w = msg->width > 0 ? msg->width : 640;

        // Precompute maps for undistortion (crucial for CPU optimization)
        cv::initUndistortRectifyMap(K_, D_, cv::Mat(), K_, cv::Size(w, h), CV_32FC1, map_x_, map_y_);
        camera_locked_ = true;
        RCLCPP_INFO(this->get_logger(), "[VISION] Undistortion map computed successfully.");
    }
}

cv::Point2f TargetVisionProcessor::extractTargetPixel(const cv::Mat &mask, cv::Mat &frame, const std::string &color_name) {
    cv::Mat morph_mask;
    cv::morphologyEx(mask, morph_mask, cv::MORPH_OPEN, morph_kernel_);
    cv::morphologyEx(morph_mask, morph_mask, cv::MORPH_CLOSE, morph_kernel_);

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(morph_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    cv::Point2f best_pixel(-1.0f, -1.0f);
    double max_area = 0.0;

    for (const auto &contour : contours) {
        double area = cv::contourArea(contour);
        if (area < 50.0) continue;

        std::vector<cv::Point> hull;
        cv::convexHull(contour, hull);
        double hull_area = cv::contourArea(hull);
        if (hull_area == 0.0) continue;

        // Ratio of contour area to hull area (solidity)
        if (area / hull_area > 0.85) {
            cv::Moments M = cv::moments(contour);
            if (M.m00 != 0.0) {
                float cX = M.m10 / M.m00;
                float cY = M.m01 / M.m00;

                if (area > max_area) {
                    max_area = area;
                    best_pixel = cv::Point2f(cX, cY);

                    if (debug_mode_) {
                        // Draw native contour in green
                        cv::drawContours(frame, std::vector<std::vector<cv::Point>>{contour}, -1, cv::Scalar(0, 255, 0), 1);

                        // Draw rotated bounding box (minAreaRect)
                        cv::RotatedRect min_rect = cv::minAreaRect(contour);
                        cv::Point2f rect_points[4];
                        min_rect.points(rect_points);
                        for (int j = 0; j < 4; j++) {
                            cv::line(frame, rect_points[j], rect_points[(j + 1) % 4], cv::Scalar(0, 255, 0), 2);
                        }

                        // Draw centroid point in red
                        cv::circle(frame, cv::Point(cX, cY), 5, cv::Scalar(0, 0, 255), -1);

                        // Put text label
                        cv::putText(frame, color_name + " TARGET", cv::Point(cX + 10, cY - 10),
                                    cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 0), 2);
                    }
                }
            }
        }
    }
    return best_pixel;
}

void TargetVisionProcessor::imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
    if (!camera_locked_) return;

    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception &e) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    cv::Mat undistorted_frame;
    cv::remap(cv_ptr->image, undistorted_frame, map_x_, map_y_, cv::INTER_LINEAR);

    cv::Mat hsv;
    cv::cvtColor(undistorted_frame, hsv, cv::COLOR_BGR2HSV);

    // Blue Target Detection
    cv::Mat blue_mask;
    cv::inRange(hsv, lower_blue_, upper_blue_, blue_mask);
    cv::Point2f blue_pixel = extractTargetPixel(blue_mask, undistorted_frame, "BLUE");

    if (blue_pixel.x >= 0.0f) {
        auto blue_msg = geometry_msgs::msg::PointStamped();
        blue_msg.header = msg->header;
        blue_msg.point.x = blue_pixel.x;
        blue_msg.point.y = blue_pixel.y;
        blue_pub_->publish(blue_msg);
    }

    // Red Target Detection
    cv::Mat red_mask1, red_mask2, red_mask;
    cv::inRange(hsv, lower_red1_, upper_red1_, red_mask1);
    cv::inRange(hsv, lower_red2_, upper_red2_, red_mask2);
    cv::bitwise_or(red_mask1, red_mask2, red_mask);
    cv::Point2f red_pixel = extractTargetPixel(red_mask, undistorted_frame, "RED");

    if (red_pixel.x >= 0.0f) {
        auto red_msg = geometry_msgs::msg::PointStamped();
        red_msg.header = msg->header;
        red_msg.point.x = red_pixel.x;
        red_msg.point.y = red_pixel.y;
        red_pub_->publish(red_msg);
    }

    if (debug_mode_) {
        cv::putText(undistorted_frame, "VISION DEBUG STREAM (C++ / UNDISTORTED)", cv::Point(10, 30),
                    cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2);
        
        auto debug_msg = cv_bridge::CvImage(msg->header, "bgr8", undistorted_frame).toImageMsg();
        debug_img_pub_->publish(*debug_msg);
    }
}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TargetVisionProcessor>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
