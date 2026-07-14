#include "uav_control/uav_control.hpp"
#include "uav_vision/segmentation.hpp"
#include "uav_vision/georeference.hpp"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/highgui.hpp>

#include <chrono>
#include <memory>

class MissionTestNode : public rclcpp::Node {
public:
    explicit MissionTestNode(Mav& mav)
        : Node("mission_test"), _mav(mav)
    {
        _img_sub = create_subscription<sensor_msgs::msg::Image>(
            "/down_camera/image", rclcpp::SensorDataQoS(),
            [this](sensor_msgs::msg::Image::SharedPtr msg) { onImage(msg); });

        _info_sub = create_subscription<sensor_msgs::msg::CameraInfo>(
            "/down_camera/camera_info", rclcpp::SensorDataQoS(),
            [this](sensor_msgs::msg::CameraInfo::SharedPtr msg) { onInfo(msg); });

        RCLCPP_INFO(get_logger(), "Waiting for camera_info...");
    }

private:
    void onInfo(const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
        if (_georef) return;
        _georef = std::make_unique<Georeference>(
            msg->k[0], msg->k[4], msg->k[2], msg->k[5]);
        _K = cv::Matx33d(msg->k[0], 0, msg->k[2],
                         0, msg->k[4], msg->k[5],
                         0, 0, 1);
        _D = cv::Mat(msg->d).clone();
        RCLCPP_INFO(get_logger(), "Camera: fx=%.1f fy=%.1f cx=%.1f cy=%.1f  D[0]=%.4f",
                    msg->k[0], msg->k[4], msg->k[2], msg->k[5],
                    msg->d.empty() ? 0.0 : msg->d[0]);
    }

    void onImage(const sensor_msgs::msg::Image::SharedPtr msg) {
        auto t = std::chrono::steady_clock::now();

        if (!_georef) return;

        auto att = _mav.getAttAt(t);
        auto ned = _mav.getNedAt(t);
        auto gps = _mav.getGPS();

        if (!att || !ned || !gps) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                "Waiting for MAVLink telemetry  att:%s ned:%s gps:%s",
                att ? "ok" : "no", ned ? "ok" : "no", gps ? "ok" : "no");
            return;
        }

        cv::Mat frame = cv_bridge::toCvCopy(msg, "bgr8")->image;
        auto centroid = _detect.detect(frame);

        _georef->rBodyNED(att->yaw_rad, att->pitch_rad, att->roll_rad);
        _georef->setReferencePoint(gps->lat, gps->lon);

        if (centroid) {
            double alt_agl = static_cast<double>(gps->alt_rel);
            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500,
                "centroid=(%.0f,%.0f)  yaw=%.1f°  pitch=%.1f°  roll=%.1f°  agl=%.1fm",
                centroid->x, centroid->y,
                att->yaw_rad * 180.0 / M_PI,
                att->pitch_rad * 180.0 / M_PI,
                att->roll_rad * 180.0 / M_PI,
                alt_agl);
            try {
                // Undistort centroid before deprojection
                std::vector<cv::Point2f> raw = {cv::Point2f(centroid->x, centroid->y)};
                std::vector<cv::Point2f> undist;
                cv::undistortPoints(raw, undist, _K, _D, cv::noArray(), _K);

                double gus = 0.0;
                auto tgt = _georef->pixelToGPS(undist[0].x, undist[0].y, alt_agl, &gus);
                double obliquity = (alt_agl > 0.1) ? (gus / alt_agl) : 99.0;
                if (obliquity > 1.5) {
                    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 500,
                        "Georef dropped: obliquity=%.2f (%.1f° off-nadir)", obliquity,
                        std::acos(1.0 / std::sqrt(obliquity)) * 180.0 / M_PI);
                } else {
                    // EMA update (α=0.2) — only when within 5m lat of overhead
                    constexpr double alpha = 0.2;
                    double dlat_m = std::abs((tgt[0] - gps->lat) * 111320.0);
                    if (!_ema.valid) { _ema.lat = tgt[0]; _ema.lon = tgt[1]; _ema.valid = true; }
                    else if (dlat_m < 5.0) {
                        _ema.lat = alpha*tgt[0] + (1-alpha)*_ema.lat;
                        _ema.lon = alpha*tgt[1] + (1-alpha)*_ema.lon;
                    }

                    RCLCPP_INFO(get_logger(),
                        "raw: (%.7f, %.7f)  ema: (%.7f, %.7f)  Δlat=%.2fm  Δlon=%.2fm  obliquity=%.2f",
                        tgt[0], tgt[1], _ema.lat, _ema.lon,
                        (tgt[0] - gps->lat) * 111320.0,
                        (tgt[1] - gps->lon) * 111320.0 * std::cos(gps->lat * M_PI / 180.0),
                        obliquity);

                    cv::putText(frame,
                        "lat:" + std::to_string(_ema.lat).substr(0, 10)
                        + " lon:" + std::to_string(_ema.lon).substr(0, 10),
                        cv::Point(10, 60), cv::FONT_HERSHEY_SIMPLEX, 0.6,
                        cv::Scalar(0, 255, 255), 2);
                }
            } catch (const std::exception& e) {
                RCLCPP_WARN(get_logger(), "Georef: %s", e.what());
            }
        }

        cv::putText(frame, centroid ? "TARGET FOUND" : "searching...",
                    cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 1.0,
                    centroid ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 0, 255), 2);

        cv::imshow("mission_test", frame);
        cv::waitKey(1);
    }

    Mav& _mav;
    Detect _detect;
    std::unique_ptr<Georeference> _georef;
    cv::Matx33d _K;
    cv::Mat _D;

    // EMA of accepted georef measurements (α=0.2 ≈ 5-sample memory)
    struct { double lat{0}, lon{0}; bool valid{false}; } _ema;

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr _img_sub;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr _info_sub;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    Mav mav("udp://:14552");

    auto node = std::make_shared<MissionTestNode>(mav);
    rclcpp::spin(node);

    cv::destroyAllWindows();
    rclcpp::shutdown();
    return 0;
}
