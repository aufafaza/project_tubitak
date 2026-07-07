#include "uav_vision/segmentation.hpp"
#include "uav_vision/georeference.hpp"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <std_msgs/msg/float64.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/highgui.hpp>
#include <Eigen/Geometry>
#include <memory>

class DetectNode : public rclcpp::Node
{
public:
    DetectNode() : Node("detect_test")
    {
        using std::placeholders::_1;

        image_sub_ = create_subscription<sensor_msgs::msg::Image>(
            "/down_camera/image", rclcpp::SensorDataQoS(),
            std::bind(&DetectNode::imageCallback, this, _1));

        info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
            "/down_camera/camera_info", rclcpp::SensorDataQoS(),
            std::bind(&DetectNode::infoCallback, this, _1));

        imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
            "/mavros/imu/data", rclcpp::SensorDataQoS(),
            std::bind(&DetectNode::imuCallback, this, _1));

        navsat_sub_ = create_subscription<sensor_msgs::msg::NavSatFix>(
            "/mavros/global_position/global", rclcpp::SensorDataQoS(),
            std::bind(&DetectNode::navsatCallback, this, _1));

        relalt_sub_ = create_subscription<std_msgs::msg::Float64>(
            "/mavros/global_position/rel_alt", rclcpp::SensorDataQoS(),
            std::bind(&DetectNode::relAltCallback, this, _1)
            );

        RCLCPP_INFO(get_logger(), "Waiting for camera_info, IMU, NavSat, and rel_alt...");
    }

private:
    void infoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
    {
        if (georef_) return;
        double fx = msg->k[0], fy = msg->k[4], cx = msg->k[2], cy = msg->k[5];
        georef_ = std::make_unique<Georeference>(fx, fy, cx, cy);
        RCLCPP_INFO(get_logger(), "Intrinsics set, fx=%.1f fy=%.1f cx=%.1f cy=%.1f",
                    fx, fy, cx, cy);
    }

    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        Eigen::Quaterniond q_enu(msg->orientation.w, msg->orientation.x,
                                 msg->orientation.y, msg->orientation.z);
                static const Eigen::Quaterniond R_enu_to_ned(0.0, 1.0/M_SQRT2, 1.0/M_SQRT2, 0.0);
        static const Eigen::Quaterniond R_flu_to_frd(0.0, 1.0, 0.0, 0.0);
        Eigen::Quaterniond q_ned = R_enu_to_ned * q_enu * R_flu_to_frd;

        Eigen::Vector3d euler = q_ned.toRotationMatrix().eulerAngles(2, 1, 0);
        yaw_   = euler[0];
        pitch_ = euler[1];
        roll_  = euler[2];
        attitude_ready_ = true;
    }

    void relAltCallback(const std_msgs::msg::Float64::SharedPtr msg){ 
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000, "alt_agl=%.2fm", msg->data);
        alt_agl_ = msg->data; 
        this->alt_ready_ = true; 
    }

    void navsatCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
    {
        lat_ = msg->latitude;
        lon_ = msg->longitude;
        if (georef_) georef_->setReferencePoint(lat_, lon_);
        gps_ready_ = true;
    }

    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        if (!georef_ || !attitude_ready_ || !gps_ready_ || !alt_ready_) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                "Waiting — intrinsics:%s attitude:%s gps:%s alt:%s",
                georef_ ? "ok" : "no",
                attitude_ready_ ? "ok" : "no",
                gps_ready_ ? "ok" : "no",
                alt_ready_ ? "ok" : "no");
            return;
        }

        cv::Mat frame = cv_bridge::toCvCopy(msg, "bgr8")->image;

        georef_->rBodyNED(yaw_, pitch_, roll_);
        georef_->setReferencePoint(lat_, lon_);

        auto centroid = detector_.detect(frame);

        if (centroid) {
            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500,
                "centroid=(%.0f,%.0f) yaw=%.1f° pitch=%.1f° roll=%.1f° agl=%.1fm",
                centroid->x, centroid->y,
                yaw_*180/M_PI, pitch_*180/M_PI, roll_*180/M_PI, alt_agl_);
            try {
                auto gps = georef_->pixelToGPS(centroid->x, centroid->y, alt_agl_);
                RCLCPP_INFO(get_logger(),
                    "Target: georef=(%.7f,%.7f) drone=(%.7f,%.7f) Δlat=%.2fm Δlon=%.2fm",
                    gps[0], gps[1], lat_, lon_,
                    (gps[0]-lat_)*111320.0,
                    (gps[1]-lon_)*111320.0*std::cos(lat_*M_PI/180.0));

                std::string label = "lat:" + std::to_string(gps[0]).substr(0, 10)
                                  + " lon:" + std::to_string(gps[1]).substr(0, 10);
                cv::putText(frame, label, cv::Point(10, 60),
                            cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 255), 2);
            } catch (const std::exception& e) {
                RCLCPP_WARN(get_logger(), "Georef: %s", e.what());
            }
        }

        cv::putText(frame, centroid ? "TARGET FOUND" : "searching...",
                    cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 1.0,
                    centroid ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 0, 255), 2);

        cv::imshow("detect_test", frame);
        cv::waitKey(1);
    }

    Detect detector_;
    std::unique_ptr<Georeference> georef_;

    double roll_{0}, pitch_{0}, yaw_{0};
    double lat_{0}, lon_{0};
    double alt_agl_{0};
    bool attitude_ready_{false}, gps_ready_{false}, alt_ready_{false};

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr navsat_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr relalt_sub_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DetectNode>());
    cv::destroyAllWindows();
    rclcpp::shutdown();
    return 0;
}
