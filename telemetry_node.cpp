#include "drone_vision/telemetry_node.hpp"
#include <unistd.h>
#include <fcntl.h>
#include <arpa/inet.h>
#include <cmath>
#include <iostream>

MavlinkTelemetryDriver::MavlinkTelemetryDriver() : Node("mavlink_telemetry_driver") {
    this->declare_parameter("payload_mass", 0.5);      // kg
    this->declare_parameter("payload_Cd", 0.0);        // Drag coefficient (Gazebo has no drag plugin)
    this->declare_parameter("payload_area", 0.00785);   // m^2 (radius 0.05m)
    this->declare_parameter("payload_rho", 1.225);     // Air density (kg/m^3)
    this->declare_parameter("servo_delay", 0.82);      // Servo latency (seconds)
    this->declare_parameter("blue_drop_offset", 0.0);  // Blue drop offset (meters)
    this->declare_parameter("red_drop_offset", 30.0);   // Red drop offset (meters)
    this->declare_parameter("wind_x", 0.0);           // East wind (m/s)
    this->declare_parameter("wind_y", 0.0);           // North wind (m/s)

    // ROS 2 QoS Configuration
    rclcpp::QoS home_qos(1);
    home_qos.transient_local();

    // ROS 2 Publishers
    pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/drone/local_pose", 10);
    vel_pub_ = this->create_publisher<geometry_msgs::msg::Vector3Stamped>("/drone/ground_velocity", 10);
    home_pub_ = this->create_publisher<sensor_msgs::msg::NavSatFix>("/drone/home_geo", home_qos);

    // ROS 2 Subscriptions
    conf_red_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
        "/target/conf_red", 10,
        std::bind(&MavlinkTelemetryDriver::confRedCallback, this, std::placeholders::_1));

    conf_blue_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
        "/target/conf_blue", 10,
        std::bind(&MavlinkTelemetryDriver::confBlueCallback, this, std::placeholders::_1));

    wp_red1_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
        "/waypoints/red1", 10,
        std::bind(&MavlinkTelemetryDriver::wpRed1Callback, this, std::placeholders::_1));

    wp_red2_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
        "/waypoints/red2", 10,
        std::bind(&MavlinkTelemetryDriver::wpRed2Callback, this, std::placeholders::_1));

    wp_blue1_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
        "/waypoints/blue1", 10,
        std::bind(&MavlinkTelemetryDriver::wpBlue1Callback, this, std::placeholders::_1));

    wp_blue2_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
        "/waypoints/blue2", 10,
        std::bind(&MavlinkTelemetryDriver::wpBlue2Callback, this, std::placeholders::_1));
    last_log_time_ = this->get_clock()->now();

    // 2.0s Timer to query home position
    home_poll_timer_ = this->create_wall_timer(
        std::chrono::seconds(2),
        std::bind(&MavlinkTelemetryDriver::queryHomePosition, this));

    // Initialize UDP Socket
    if (initSocket()) {
        RCLCPP_INFO(this->get_logger(), "Telemetry socket bound to UDP port 14550 successfully.");
        // Spawn Background Ingestion and Heartbeat threads
        receiver_thread_ = std::thread(&MavlinkTelemetryDriver::mavlinkReceiverLoop, this);
        heartbeat_thread_ = std::thread(&MavlinkTelemetryDriver::heartbeatLoop, this);
    } else {
        RCLCPP_FATAL(this->get_logger(), "CRITICAL: Failed to bind telemetry UDP socket on port 14550.");
    }
}

MavlinkTelemetryDriver::~MavlinkTelemetryDriver() {
    run_threads_ = false;
    if (socket_fd_ >= 0) {
        close(socket_fd_);
    }
    if (receiver_thread_.joinable()) {
        receiver_thread_.join();
    }
    if (heartbeat_thread_.joinable()) {
        heartbeat_thread_.join();
    }
}

bool MavlinkTelemetryDriver::initSocket() {
    socket_fd_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (socket_fd_ < 0) {
        return false;
    }

    int opt = 1;
    setsockopt(socket_fd_, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    int flags = fcntl(socket_fd_, F_GETFL, 0);
    fcntl(socket_fd_, F_SETFL, flags | O_NONBLOCK);

    std::memset(&local_addr_, 0, sizeof(local_addr_));
    local_addr_.sin_family = AF_INET;
    local_addr_.sin_addr.s_addr = INADDR_ANY;
    local_addr_.sin_port = htons(14550);

    if (bind(socket_fd_, (struct sockaddr *)&local_addr_, sizeof(local_addr_)) < 0) {
        close(socket_fd_);
        return false;
    }

    return true;
}

void MavlinkTelemetryDriver::sendMavlinkMessage(const mavlink_message_t &msg) {
    std::lock_guard<std::mutex> lock(socket_mutex_);
    if (socket_fd_ < 0 || !remote_addr_cached_) {
        return;
    }

    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);

    ssize_t sent = sendto(socket_fd_, buf, len, 0, (struct sockaddr *)&remote_addr_, sizeof(remote_addr_));
    if (sent < 0) {
        RCLCPP_ERROR(this->get_logger(), "[TELEMETRY C++] sendto failed to %s:%d: %s",
                     inet_ntoa(remote_addr_.sin_addr), ntohs(remote_addr_.sin_port), strerror(errno));
    }
}

void MavlinkTelemetryDriver::heartbeatLoop() {
    while (run_threads_ && rclcpp::ok()) {
        if (remote_addr_cached_) {
            mavlink_message_t msg;
            mavlink_msg_heartbeat_pack(
                system_id_, component_id_, &msg,
                MAV_TYPE_GCS, MAV_AUTOPILOT_INVALID, MAV_MODE_MANUAL_ARMED, 0, MAV_STATE_ACTIVE
            );
            sendMavlinkMessage(msg);
        }
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
}

void MavlinkTelemetryDriver::mavlinkReceiverLoop() {
    uint8_t rx_buf[2048];
    struct sockaddr_in client_addr;
    mavlink_message_t msg;
    mavlink_status_t status;
    while (run_threads_ && rclcpp::ok()) {
        socklen_t addr_len = sizeof(client_addr);
        ssize_t bytes_received = recvfrom(
            socket_fd_, rx_buf, sizeof(rx_buf), 0,
            (struct sockaddr *)&client_addr, &addr_len
        );

        if (bytes_received > 0) {
            if (!remote_addr_cached_) {
                std::lock_guard<std::mutex> lock(socket_mutex_);
                remote_addr_ = client_addr;
                remote_addr_cached_ = true;
                RCLCPP_INFO(this->get_logger(), "Auto-detected autopilot socket source: %s:%d",
                            inet_ntoa(remote_addr_.sin_addr), ntohs(remote_addr_.sin_port));
            }

            for (ssize_t i = 0; i < bytes_received; ++i) {
                if (mavlink_parse_char(MAVLINK_COMM_0, rx_buf[i], &msg, &status)) {
                    if (!target_ids_resolved_ && msg.msgid == MAVLINK_MSG_ID_HEARTBEAT) {
                        mavlink_heartbeat_t hb;
                        mavlink_msg_heartbeat_decode(&msg, &hb);
                        if (hb.type != MAV_TYPE_GCS && msg.sysid != 255) {
                            target_system_id_ = msg.sysid;
                            target_component_id_ = msg.compid;
                            target_ids_resolved_ = true;

                            sendParameterSet("SERVO8_MIN", 800.0f, MAV_PARAM_TYPE_REAL32);
                            sendParameterSet("SR0_POSITION", 50.0f, MAV_PARAM_TYPE_INT16);
                            sendParameterSet("SR0_EXTRA1", 50.0f, MAV_PARAM_TYPE_INT16);
                            sendParameterSet("SR0_RAW_CTRL", 10.0f, MAV_PARAM_TYPE_INT16);
                            sendParameterSet("SR1_POSITION", 50.0f, MAV_PARAM_TYPE_INT16);
                            sendParameterSet("SR1_EXTRA1", 50.0f, MAV_PARAM_TYPE_INT16);
                            sendParameterSet("SR1_RAW_CTRL", 10.0f, MAV_PARAM_TYPE_INT16);
                            requestDataStream(MAV_DATA_STREAM_POSITION, 50);
                            requestDataStream(MAV_DATA_STREAM_EXTRA1, 50);
                            requestMessageInterval(MAVLINK_MSG_ID_GLOBAL_POSITION_INT, 100000);
                            requestMessageInterval(MAVLINK_MSG_ID_SERVO_OUTPUT_RAW, 100000);
                            requestMessageInterval(MAVLINK_MSG_ID_WIND, 200000);

                            RCLCPP_INFO(this->get_logger(), "Handshake complete. Telemetry locked on system ID %d.", target_system_id_);
                        }
                    }
                    handleMavlinkMessage(msg);
                }
            }
        } else {
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
    }
}

void MavlinkTelemetryDriver::handleMavlinkMessage(const mavlink_message_t &msg) {
    if (msg.msgid == MAVLINK_MSG_ID_HEARTBEAT) {
        mavlink_heartbeat_t hb;
        mavlink_msg_heartbeat_decode(&msg, &hb);
        if (vehicle_type_ == 0) {
            vehicle_type_ = hb.type;
            if (vehicle_type_ == MAV_TYPE_FIXED_WING || vehicle_type_ == MAV_TYPE_VTOL_TILTROTOR) {
                model_prefix_ = "mini_talon_vtail";
            } else {
                model_prefix_ = "iris_with_gimbal";
            }
            servo_pub_ = this->create_publisher<std_msgs::msg::Float64>("/" + model_prefix_ + "/payload/release", 10);
            RCLCPP_INFO(this->get_logger(), "Vehicle model detected as: %s", model_prefix_.c_str());
        }
    }
    
    switch (msg.msgid) {
        case MAVLINK_MSG_ID_ATTITUDE:
            handleAttitude(msg);
            break;
        case MAVLINK_MSG_ID_LOCAL_POSITION_NED:
            handleLocalPositionNed(msg);
            break;
        case MAVLINK_MSG_ID_HOME_POSITION:
            handleHomePosition(msg);
            break;
        case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:
            handleGlobalPositionInt(msg);
            break;
        case MAVLINK_MSG_ID_SERVO_OUTPUT_RAW:
            handleServoOutputRaw(msg);
            break;
        case MAVLINK_MSG_ID_MISSION_REQUEST: {
            mavlink_mission_request_t req;
            mavlink_msg_mission_request_decode(&msg, &req);
            std::lock_guard<std::mutex> lock(mission_mutex_);
            requested_seq_ = req.seq;
            has_mission_request_ = true;
            mission_cv_.notify_all();
            break;
        }
        case MAVLINK_MSG_ID_MISSION_REQUEST_INT: {
            mavlink_mission_request_int_t req;
            mavlink_msg_mission_request_int_decode(&msg, &req);
            std::lock_guard<std::mutex> lock(mission_mutex_);
            requested_seq_ = req.seq;
            has_mission_request_ = true;
            mission_cv_.notify_all();
            break;
        }
        case MAVLINK_MSG_ID_MISSION_ACK: {
            mavlink_mission_ack_t ack;
            mavlink_msg_mission_ack_decode(&msg, &ack);
            std::lock_guard<std::mutex> lock(mission_mutex_);
            ack_type_ = ack.type;
            has_mission_ack_ = true;
            mission_cv_.notify_all();
            break;
        }
        case MAVLINK_MSG_ID_WIND: {
            mavlink_wind_t wind;
            mavlink_msg_wind_decode(&msg, &wind);
            double wind_from_rad = wind.direction * M_PI / 180.0;
            double wind_to_rad = wind_from_rad + M_PI; 
            live_wind_x_ = wind.speed * std::sin(wind_to_rad);
            live_wind_y_ = wind.speed * std::cos(wind_to_rad);
            break;
        }
    }
}

void MavlinkTelemetryDriver::syncClocks(uint32_t time_boot_ms) {
    double current_ros_time = this->get_clock()->now().nanoseconds() / 1e9;
    if (current_ros_time <= 1e-3) return;

    double instant_offset = current_ros_time - (time_boot_ms / 1000.0);
    bool use_sim_time = false;
    this->get_parameter("use_sim_time", use_sim_time);
    
    if (use_sim_time) {
        if (!clock_initialized_) {
            clock_offset_ = instant_offset;
            clock_initialized_ = true;
        }
        return;
    }

    if (!clock_initialized_) {
        clock_offset_ = instant_offset;
        clock_initialized_ = true;
    } else {
        clock_offset_ = 0.95 * clock_offset_ + 0.05 * instant_offset;
    }
}

rclcpp::Time MavlinkTelemetryDriver::getSyncedRosTime(uint32_t time_boot_ms) {
    double current_ros_time = this->get_clock()->now().nanoseconds() / 1e9;
    if (!clock_initialized_ && current_ros_time > 1e-3) {
        clock_offset_ = current_ros_time - (time_boot_ms / 1000.0);
        clock_initialized_ = true;
    }
    double t_sec = (time_boot_ms / 1000.0) + clock_offset_;
    uint32_t sec = static_cast<uint32_t>(t_sec);
    uint32_t nsec = static_cast<uint32_t>((t_sec - sec) * 1e9);
    return rclcpp::Time(sec, nsec);
}

void MavlinkTelemetryDriver::handleHomePosition(const mavlink_message_t &msg) {
    mavlink_home_position_t home;
    mavlink_msg_home_position_decode(&msg, &home);
    home_lat_ = home.latitude / 1e7;
    home_lon_ = home.longitude / 1e7;
    home_from_master_ = true;
    home_locked_ = true;
    RCLCPP_INFO(this->get_logger(), "[TELEMETRY C++] Home Position Acquired: Lat=%.6f, Lon=%.6f", home_lat_, home_lon_);
    
    // FORCE TELEMETRY STREAM RATES OPEN UPON STABLE HOME LOCK
    requestDataStream(MAV_DATA_STREAM_POSITION, 50);
    requestDataStream(MAV_DATA_STREAM_EXTRA1, 50);

    queryHomePosition();
}


void MavlinkTelemetryDriver::handleGlobalPositionInt(const mavlink_message_t &msg) {
    mavlink_global_position_int_t gp;
    mavlink_msg_global_position_int_decode(&msg, &gp);
    syncClocks(gp.time_boot_ms);

    if (!home_from_master_ && !home_locked_ && gp.lat != 0 && gp.lon != 0) {
        home_lat_ = gp.lat / 1e7;
        home_lon_ = gp.lon / 1e7;
        home_locked_ = true;
        RCLCPP_WARN(this->get_logger(), "[TELEMETRY C++ FALLBACK] Captured mid-flight position framework lock.");
        queryHomePosition();
    }
}

// -----------------------------------------------------------------
// 50Hz TELEMETRY RECEPTION ENGINE WITH TRACKER INTEGRATION
// -----------------------------------------------------------------
void MavlinkTelemetryDriver::handleLocalPositionNed(const mavlink_message_t &msg) {
    mavlink_local_position_ned_t lp;
    mavlink_msg_local_position_ned_decode(&msg, &lp);

    syncClocks(lp.time_boot_ms);
    rclcpp::Time stamp = getSyncedRosTime(lp.time_boot_ms);

    curr_north_ = lp.x;
    curr_east_ = lp.y;
    curr_alt_ = -lp.z;
    curr_speed_ = std::sqrt(lp.vx * lp.vx + lp.vy * lp.vy + lp.vz * lp.vz);
    
    curr_vx_ = lp.vy;  // East
    curr_vy_ = lp.vx;  // North
    curr_vz_ = -lp.vz; // Up
    
    cached_pose_.header.stamp = stamp;
    cached_pose_.header.frame_id = "world_enu";
    cached_pose_.pose.position.x = lp.y;   
    cached_pose_.pose.position.y = lp.x;   
    cached_pose_.pose.position.z = -lp.z;  
    cached_pose_initialized_ = true;

    if (vel_pub_) {
        geometry_msgs::msg::Vector3Stamped vel_msg;
        vel_msg.header.stamp = stamp;
        vel_msg.header.frame_id = "world_enu";
        vel_msg.vector.x = curr_vx_;  
        vel_msg.vector.y = curr_vy_;  
        vel_msg.vector.z = curr_vz_; 
        vel_pub_->publish(vel_msg);
    }

    static rclcpp::Time last_debug_print = this->get_clock()->now();
    bool print_diagnostics = (this->get_clock()->now() - last_debug_print).seconds() > 0.5;
    if (print_diagnostics) {
        last_debug_print = this->get_clock()->now();
    }

    if (!servo_pub_) {
        servo_pub_ = this->create_publisher<std_msgs::msg::Float64>("/" + model_prefix_ + "/payload/release", 10);
    }

    // =================================================================
    // PHASE 1: BLUE TRACKER (Executed First)
    // =================================================================
    if (has_blue_target_ && !blue_drop_completed_) {
        if (!blue_drop_primed_) {
            Eigen::Vector2d target_enu = gpsToEnu(final_blue_lat_, final_blue_lon_);
            double dist_to_target = std::sqrt(std::pow(target_enu.y() - curr_east_, 2) + 
                                              std::pow(target_enu.x() - curr_north_, 2));
            if (dist_to_target < 150.0) {
                blue_drop_primed_ = true;
                RCLCPP_INFO(this->get_logger(), "[TRACKER BLUE] Within 150m of target. Ballistic tracking ENGAGED.");
            }
        }

        if (blue_drop_primed_) {
            Eigen::Vector2d release_point = calculateBallisticRelease(final_blue_lat_, final_blue_lon_, true);
            double release_north = release_point.x();
            double release_east = release_point.y();

            double distance_to_release = std::sqrt(std::pow(release_east - curr_east_, 2) + 
                                                   std::pow(release_north - curr_north_, 2));

            double track_vector_x = release_east - curr_east_;
            double track_vector_y = release_north - curr_north_;
            double dot_product = track_vector_x * curr_vx_ + track_vector_y * curr_vy_;

            if (print_diagnostics) {
                RCLCPP_INFO(this->get_logger(), "[RUN BLUE] Dist to Release: %.2fm | Dot: %.2f", distance_to_release, dot_product);
            }

            if (distance_to_release < 1.0 || (dot_product < 0.0 && distance_to_release < 4.0)) {
                sendServoOutput(8, 2000.0f); 
                if (servo_pub_) {
                    std_msgs::msg::Float64 release_msg;
                    release_msg.data = 1.0; 
                    servo_pub_->publish(release_msg);
                }
                blue_drop_completed_ = true;
                RCLCPP_INFO(this->get_logger(), "[RELEASE] BLUE payload drop commanded via dynamic logic.");
            }
        }
    }
    // =================================================================
    // PHASE 2: RED TRACKER (Strictly blocked until Phase 1 is complete)
    // =================================================================
    else if (blue_drop_completed_ && has_red_target_ && !red_dropped_) {
        if (!red_drop_primed_) {
            Eigen::Vector2d target_enu = gpsToEnu(final_red_lat_, final_red_lon_);
            double dist_to_target = std::sqrt(std::pow(target_enu.y() - curr_east_, 2) + 
                                              std::pow(target_enu.x() - curr_north_, 2));
            if (dist_to_target < 150.0) {
                red_drop_primed_ = true;
                RCLCPP_INFO(this->get_logger(), "[TRACKER RED] Within 150m of target. Ballistic tracking ENGAGED.");
            }
        }

        if (red_drop_primed_) {
            Eigen::Vector2d release_point = calculateBallisticRelease(final_red_lat_, final_red_lon_, false);
            double release_north = release_point.x();
            double release_east = release_point.y();

            double distance_to_release = std::sqrt(std::pow(release_east - curr_east_, 2) + 
                                                   std::pow(release_north - curr_north_, 2));

            double track_vector_x = release_east - curr_east_;
            double track_vector_y = release_north - curr_north_;
            double dot_product = track_vector_x * curr_vx_ + track_vector_y * curr_vy_;

            if (print_diagnostics) {
                RCLCPP_INFO(this->get_logger(), "[RUN RED] Dist to Release: %.2fm | Dot: %.2f", distance_to_release, dot_product);
            }

            if (distance_to_release < 1.0 || (dot_product < 0.0 && distance_to_release < 4.0)) {
                sendServoOutput(8, 1200.0f); 
                if (servo_pub_) {
                    std_msgs::msg::Float64 release_msg;
                    release_msg.data = 0.2; 
                    servo_pub_->publish(release_msg);
                }
                red_dropped_ = true;
                RCLCPP_INFO(this->get_logger(), "[RELEASE] RED payload drop commanded via dynamic logic.");
            }
        }
    }
}

void MavlinkTelemetryDriver::handleAttitude(const mavlink_message_t &msg) {
    mavlink_attitude_t att;
    mavlink_msg_attitude_decode(&msg, &att);

    syncClocks(att.time_boot_ms);
    curr_yaw_ = att.yaw;

    if (!cached_pose_initialized_) return;

    rclcpp::Time stamp = getSyncedRosTime(att.time_boot_ms);
    cached_pose_.header.stamp = stamp;

    Eigen::Matrix3d R_ned_frd = (
        Eigen::AngleAxisd(att.yaw, Eigen::Vector3d::UnitZ()) *
        Eigen::AngleAxisd(att.pitch, Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(att.roll, Eigen::Vector3d::UnitX())
    ).toRotationMatrix();

    Eigen::Matrix3d R_ned_to_enu;
    R_ned_to_enu << 0.0, 1.0,  0.0,
                    1.0, 0.0,  0.0,
                    0.0, 0.0, -1.0;

    Eigen::Matrix3d R_frd_to_flu;
    R_frd_to_flu << 1.0,  0.0,  0.0,
                    0.0, -1.0,  0.0,
                    0.0,  0.0, -1.0;

    Eigen::Matrix3d R_enu_flu = R_ned_to_enu * R_ned_frd * R_frd_to_flu.transpose();
    Eigen::Quaterniond q(R_enu_flu);

    cached_pose_.pose.orientation.x = q.x();
    cached_pose_.pose.orientation.y = q.y();
    cached_pose_.pose.orientation.z = q.z();
    cached_pose_.pose.orientation.w = q.w();

    pose_pub_->publish(cached_pose_);

    msg_count_++;
    auto current_time = this->get_clock()->now();
    double elapsed = (current_time - last_log_time_).nanoseconds() / 1e9;
    if (elapsed >= 5.0) {
        RCLCPP_INFO(this->get_logger(), "[TELEMETRY C++] Stream Active | Rx rate: %.1f Hz", msg_count_ / elapsed);
        msg_count_ = 0;
        last_log_time_ = current_time;
    }
}

void MavlinkTelemetryDriver::handleServoOutputRaw(const mavlink_message_t &msg) {
    mavlink_servo_output_raw_t servo;
    mavlink_msg_servo_output_raw_decode(&msg, &servo);

    uint16_t pwm = servo.servo8_raw;
    if (pwm > 0) {
        double val = (pwm - 1000.0) / 1000.0;
        if (servo_pub_) {
            std_msgs::msg::Float64 release_msg;
            release_msg.data = val;
            servo_pub_->publish(release_msg);
        }

        // Detect actual drops via servo state changes
        if (val > 0.75 && !blue_drop_logged_) {
            blue_drop_logged_ = true;
            blue_drop_completed_ = true; // Ensure completed state aligns
            calculateAndLogDropError("blue");
        } else if (val < 0.25 && std::abs(val) > 0.02 && !red_drop_logged_) {
            red_drop_logged_ = true;
            red_dropped_ = true; // Ensure completed state aligns
            calculateAndLogDropError("red");
        }
    }
}

void MavlinkTelemetryDriver::queryHomePosition() {
    if (socket_fd_ < 0 || !remote_addr_cached_) return;
    
    // Periodically re-request data streams to guarantee position and attitude telemetry streams stay active
    requestDataStream(MAV_DATA_STREAM_POSITION, 50);
    requestDataStream(MAV_DATA_STREAM_EXTRA1, 50);

    if (!home_from_master_) {
        RCLCPP_INFO(this->get_logger(), "[TELEMETRY C++] Requesting HOME_POSITION from autopilot...");
        mavlink_message_t request_msg;
        mavlink_msg_command_long_pack(
            system_id_, component_id_, &request_msg,
            target_system_id_, target_component_id_,
            MAV_CMD_REQUEST_MESSAGE, 0,
            242.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f
        );
        sendMavlinkMessage(request_msg);
    }

    if (home_locked_) {
        sensor_msgs::msg::NavSatFix home_msg;
        home_msg.header.stamp = this->get_clock()->now();
        home_msg.latitude = home_lat_;
        home_msg.longitude = home_lon_;
        home_pub_->publish(home_msg);
    }
}

// -----------------------------------------------------------------
// PURE BALLISTIC PREDICTOR - RETURNING ABSOLUTE ENU LOCAL COORDINATE
// -----------------------------------------------------------------
// -----------------------------------------------------------------
// PURE BALLISTIC PREDICTOR - WITH LIVE TELEMETRY LOGGING
// -----------------------------------------------------------------
Eigen::Vector2d MavlinkTelemetryDriver::calculateBallisticRelease(double target_lat, double target_lon, bool is_blue) {
    double m = this->get_parameter("payload_mass").as_double();
    double Cd = this->get_parameter("payload_Cd").as_double();
    double A = this->get_parameter("payload_area").as_double();
    double rho = this->get_parameter("payload_rho").as_double();
    double t_servo_delay = this->get_parameter("servo_delay").as_double();
    double drop_offset = is_blue ? this->get_parameter("blue_drop_offset").as_double()
                                 : this->get_parameter("red_drop_offset").as_double();
    double wind_x = live_wind_x_.load(); // East Wind component
    double wind_y = live_wind_y_.load(); // North Wind component

    double k = 0.5 * rho * Cd * A / m;
    double g = 9.80665;

    double sim_x = 0.0, sim_y = 0.0, sim_z = curr_alt_;
    double sim_vx = curr_vx_, sim_vy = curr_vy_, sim_vz = curr_vz_;
    double dt = 0.01;
    double t_fall = 0.0;

    // Numerical forward integration loop
    while (sim_z > 0.0 && t_fall < 15.0) {
        double v_rel_x = sim_vx - wind_x;
        double v_rel_y = sim_vy - wind_y;
        double v_rel_z = sim_vz;
        double v_rel = std::sqrt(v_rel_x * v_rel_x + v_rel_y * v_rel_y + v_rel_z * v_rel_z);

        sim_vx += (-k * v_rel * v_rel_x) * dt;
        sim_vy += (-k * v_rel * v_rel_y) * dt;
        sim_vz += (-g - k * v_rel * v_rel_z) * dt;

        sim_x += sim_vx * dt;
        sim_y += sim_vy * dt;
        sim_z += sim_vz * dt;
        t_fall += dt;
    }

    Eigen::Vector2d target_enu;
    if (is_blue) {
        target_enu = Eigen::Vector2d(150.0, 60.0); // (North, East)
    } else {
        target_enu = Eigen::Vector2d(120.0, 0.0);  // (North, East)
    }
    
    // Explicit transformations matching ENU layout
    double unit_vx = (curr_speed_ > 0.1) ? (curr_vx_ / curr_speed_) : 0.0;
    double unit_vy = (curr_speed_ > 0.1) ? (curr_vy_ / curr_speed_) : 0.0;

    double release_east = target_enu.y() - sim_x - (curr_vx_ * t_servo_delay) + (unit_vx * drop_offset);
    double release_north = target_enu.x() - sim_y - (curr_vy_ * t_servo_delay) + (unit_vy * drop_offset);

    // Rate-limited print inside the solver calculation block itself
    static rclcpp::Time last_solver_print = this->get_clock()->now();
    if ((this->get_clock()->now() - last_solver_print).seconds() > 1.0) {
        last_solver_print = this->get_clock()->now();
        RCLCPP_INFO(this->get_logger(), 
            "[SOLVER INFO] Sim Fall Time: %.2fs | Advance Delta: E_%.2fm, N_%.2fm | Ground Speed: %.2f m/s", 
            t_fall, sim_x, sim_y, curr_speed_);
    }

    return Eigen::Vector2d(release_north, release_east);
}

void MavlinkTelemetryDriver::calculateAndLogDropError(const std::string &color) {
    double t_lat = 0.0, t_lon = 0.0;
    if (color == "blue") {
        t_lat = final_blue_lat_;
        t_lon = final_blue_lon_;
    } else {
        t_lat = final_red_lat_;
        t_lon = final_red_lon_;
    }

    if (t_lat == 0.0 || !home_locked_) {
        RCLCPP_ERROR(this->get_logger(), "[DROP ERROR] Target %s or Home coordinates not set!", color.c_str());
        return;
    }

    Eigen::Vector2d target_enu;
    if (color == "blue") {
        target_enu = Eigen::Vector2d(150.0, 60.0); // (North, East)
    } else {
        target_enu = Eigen::Vector2d(120.0, 0.0);  // (North, East)
    }

    double m = this->get_parameter("payload_mass").as_double();
    double Cd = this->get_parameter("payload_Cd").as_double();
    double A = this->get_parameter("payload_area").as_double();
    double rho = this->get_parameter("payload_rho").as_double();
    double wind_x = live_wind_x_.load();
    double wind_y = live_wind_y_.load();

    double k = 0.5 * rho * Cd * A / m;
    double g = 9.80665;

    double sim_x = 0.0, sim_y = 0.0, sim_z = curr_alt_;
    double sim_vx = curr_vx_, sim_vy = curr_vy_, sim_vz = curr_vz_;
    double dt = 0.01;
    double t_fall = 0.0;

    while (sim_z > 0.0 && t_fall < 15.0) {
        double v_rel_x = sim_vx - wind_x;
        double v_rel_y = sim_vy - wind_y;
        double v_rel_z = sim_vz;
        double v_rel = std::sqrt(v_rel_x * v_rel_x + v_rel_y * v_rel_y + v_rel_z * v_rel_z);

        sim_vx += (-k * v_rel * v_rel_x) * dt;
        sim_vy += (-k * v_rel * v_rel_y) * dt;
        sim_vz += (-g - k * v_rel * v_rel_z) * dt;

        sim_x += sim_vx * dt;
        sim_y += sim_vy * dt;
        sim_z += sim_vz * dt;
        t_fall += dt;
    }

    double landing_east = curr_east_ + sim_x;
    double landing_north = curr_north_ + sim_y;

    double land_lat = 0.0, land_lon = 0.0;
    enuToGps(landing_north, landing_east, land_lat, land_lon);

    double error = std::sqrt(std::pow(landing_east - target_enu.y(), 2) + std::pow(landing_north - target_enu.x(), 2));

    RCLCPP_INFO(this->get_logger(),
        "\n=======================================================\n"
        "[TELEMETRY DROP REPORT - %s]\n"
        "Release point ENU: E=%.2fm, N=%.2fm, Alt=%.2fm\n"
        "Release speed ENU: vx=%.2f m/s, vy=%.2f m/s, vz=%.2f m/s\n"
        "Estimated landing point ENU: E=%.2fm, N=%.2fm\n"
        "Estimated landing GPS: Lat=%.6f, Lon=%.6f\n"
        "Target location GPS: Lat=%.6f, Lon=%.6f\n"
        "Target position ENU: E=%.2fm, N=%.2fm\n"
        "Calculated Drop Error (Miss Distance): %.3f meters\n"
        "=======================================================\n",
        (color == "blue" ? "BLUE" : "RED"), curr_east_, curr_north_, curr_alt_, curr_vx_, curr_vy_, curr_vz_,
        landing_east, landing_north, land_lat, land_lon, t_lat, t_lon, target_enu.y(), target_enu.x(), error);
}

Eigen::Vector2d MavlinkTelemetryDriver::gpsToEnu(double lat, double lon) {
    double R_EARTH = 6378137.0;
    double cos_lat = std::cos(M_PI * home_lat_ / 180.0);
    
    // x() corresponds to North, y() corresponds to East based on your enuToGps usage
    double north = (lat - home_lat_) * (M_PI / 180.0) * R_EARTH;
    double east = (lon - home_lon_) * (M_PI / 180.0) * R_EARTH * cos_lat;
    
    return Eigen::Vector2d(north, east);
}

void MavlinkTelemetryDriver::enuToGps(double north, double east, double &lat, double &lon) {
    double R_EARTH = 6378137.0;
    double cos_lat = std::cos(M_PI * home_lat_ / 180.0);
    lat = home_lat_ + (north / R_EARTH * 180.0 / M_PI);
    lon = home_lon_ + (east / (R_EARTH * cos_lat) * 180.0 / M_PI);
}

void MavlinkTelemetryDriver::confRedCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
    // Simply accept the target coordinates immediately without sensor noise variance gating
    if (!has_red_target_ && home_locked_ && !red_dropped_) {
        final_red_lat_ = msg->latitude;
        final_red_lon_ = msg->longitude;
        red_target_enu_ = gpsToEnu(final_red_lat_, final_red_lon_);
        has_red_target_ = true;
        RCLCPP_INFO(this->get_logger(), "[TARGET SET] RED coordinate logged: Lat=%.6f, Lon=%.6f", final_red_lat_, final_red_lon_);
    }
}

void MavlinkTelemetryDriver::confBlueCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
    if (!has_blue_target_ && home_locked_ && !blue_drop_completed_) {
        final_blue_lat_ = msg->latitude;
        final_blue_lon_ = msg->longitude;
        blue_target_enu_ = gpsToEnu(final_blue_lat_, final_blue_lon_);
        has_blue_target_ = true;
        RCLCPP_INFO(this->get_logger(), "[TARGET SET] BLUE coordinate logged: Lat=%.6f, Lon=%.6f", final_blue_lat_, final_blue_lon_);
    }
}

void MavlinkTelemetryDriver::wpRed1Callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
    if (home_locked_) {
        wp_red1_lat_ = msg->latitude;
        wp_red1_lon_ = msg->longitude;
        has_wp_red1_ = true;
        waypoint_node_mode_ = true;
        RCLCPP_INFO(this->get_logger(), "[TELEMETRY C++] Received RED1 from waypoint node: Lat=%.6f, Lon=%.6f", wp_red1_lat_, wp_red1_lon_);
        checkAndUploadMission();
    }
}

void MavlinkTelemetryDriver::wpRed2Callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
    if (home_locked_) {
        wp_red2_lat_ = msg->latitude;
        wp_red2_lon_ = msg->longitude;
        has_wp_red2_ = true;
        waypoint_node_mode_ = true;
        RCLCPP_INFO(this->get_logger(), "[TELEMETRY C++] Received RED2 from waypoint node: Lat=%.6f, Lon=%.6f", wp_red2_lat_, wp_red2_lon_);
        checkAndUploadMission();
    }
}

void MavlinkTelemetryDriver::wpBlue1Callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
    if (home_locked_) {
        wp_blue1_lat_ = msg->latitude;
        wp_blue1_lon_ = msg->longitude;
        has_wp_blue1_ = true;
        waypoint_node_mode_ = true;
        RCLCPP_INFO(this->get_logger(), "[TELEMETRY C++] Received BLUE1 from waypoint node: Lat=%.6f, Lon=%.6f", wp_blue1_lat_, wp_blue1_lon_);
        checkAndUploadMission();
    }
}

void MavlinkTelemetryDriver::wpBlue2Callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
    if (home_locked_) {
        wp_blue2_lat_ = msg->latitude;
        wp_blue2_lon_ = msg->longitude;
        has_wp_blue2_ = true;
        waypoint_node_mode_ = true;
        RCLCPP_INFO(this->get_logger(), "[TELEMETRY C++] Received BLUE2 from waypoint node: Lat=%.6f, Lon=%.6f", wp_blue2_lat_, wp_blue2_lon_);
        checkAndUploadMission();
    }
}

void MavlinkTelemetryDriver::checkAndUploadMission() {
    bool can_upload = false;
    bool using_wp_node = false;

    if (waypoint_node_mode_) {
        if (has_wp_red1_ && has_wp_red2_ && has_wp_blue1_ && has_wp_blue2_) {
            can_upload = true;
            using_wp_node = true;
        }
    } else {
        if (has_red_target_ && has_blue_target_) {
            can_upload = true;
        }
    }

    if (can_upload && !mission_plan_uploaded_ && !uploading_mission_) {
        uploading_mission_ = true;
        std::thread([this, using_wp_node]() {
            double art_blue_lat, art_blue_lon;
            double blue_lat, blue_lon;
            double new_blue_lat, new_blue_lon;
            double art_red_lat, art_red_lon;
            double red_lat, red_lon;
            double new_red_lat, new_red_lon;

            if (using_wp_node) {
                RCLCPP_INFO(this->get_logger(), "[TELEMETRY C++] Both targets resolved via external waypoint node. Generating tactical path...");
                art_blue_lat = wp_blue2_lat_;
                art_blue_lon = wp_blue2_lon_;
                blue_lat = wp_blue1_lat_;
                blue_lon = wp_blue1_lon_;

                Eigen::Vector2d blue_enu = gpsToEnu(blue_lat, blue_lon);
                Eigen::Vector2d art_blue_enu = gpsToEnu(art_blue_lat, art_blue_lon);
                double d_north_blue = blue_enu.x() - art_blue_enu.x();
                double d_east_blue = blue_enu.y() - art_blue_enu.y();
                double dist_blue = std::sqrt(d_north_blue * d_north_blue + d_east_blue * d_east_blue);
                double u_n_blue = (dist_blue > 1.0) ? (d_north_blue / dist_blue) : 1.0;
                double u_e_blue = (dist_blue > 1.0) ? (d_east_blue / dist_blue) : 0.0;
                double new_blue_north = blue_enu.x() + 30.0 * u_n_blue;
                double new_blue_east = blue_enu.y() + 30.0 * u_e_blue;
                enuToGps(new_blue_north, new_blue_east, new_blue_lat, new_blue_lon);

                art_red_lat = wp_red2_lat_;
                art_red_lon = wp_red2_lon_;
                red_lat = wp_red1_lat_;
                red_lon = wp_red1_lon_;

                Eigen::Vector2d red_enu = gpsToEnu(red_lat, red_lon);
                Eigen::Vector2d art_red_enu = gpsToEnu(art_red_lat, art_red_lon);
                double d_north_red = red_enu.x() - art_red_enu.x();
                double d_east_red = red_enu.y() - art_red_enu.y();
                double dist_red = std::sqrt(d_north_red * d_north_red + d_east_red * d_east_red);
                double u_n_red = (dist_red > 1.0) ? (d_north_red / dist_red) : 1.0;
                double u_e_red = (dist_red > 1.0) ? (d_east_red / dist_red) : 0.0;
                double new_red_north = red_enu.x() + 30.0 * u_n_red;
                double new_red_east = red_enu.y() + 30.0 * u_e_red;
                enuToGps(new_red_north, new_red_east, new_red_lat, new_red_lon);
            } else {
                RCLCPP_INFO(this->get_logger(), "[TELEMETRY C++] Both targets resolved internally. Starting C++ tactical path mission generator...");
                Eigen::Vector2d red_enu = gpsToEnu(final_red_lat_, final_red_lon_);
                Eigen::Vector2d blue_enu = gpsToEnu(final_blue_lat_, final_blue_lon_);

                blue_lat = final_blue_lat_;
                blue_lon = final_blue_lon_;
                red_lat = final_red_lat_;
                red_lon = final_red_lon_;

                double y_perp = std::cos(curr_yaw_);
                double x_perp = -std::sin(curr_yaw_);
                double art_blue_north = blue_enu.x() + 200.0 * y_perp;
                double art_blue_east = blue_enu.y() + 200.0 * x_perp;
                enuToGps(art_blue_north, art_blue_east, art_blue_lat, art_blue_lon);

                double d_north_blue = blue_enu.x() - art_blue_north;
                double d_east_blue = blue_enu.y() - art_blue_east;
                double dist_blue = std::sqrt(d_north_blue * d_north_blue + d_east_blue * d_east_blue);
                double u_n_blue = (dist_blue > 1.0) ? (d_north_blue / dist_blue) : 1.0;
                double u_e_blue = (dist_blue > 1.0) ? (d_east_blue / dist_blue) : 0.0;
                double new_blue_north = blue_enu.x() + 30.0 * u_n_blue;
                double new_blue_east = blue_enu.y() + 30.0 * u_e_blue;
                enuToGps(new_blue_north, new_blue_east, new_blue_lat, new_blue_lon);

                double d_north = red_enu.x() - blue_enu.x();
                double d_east = red_enu.y() - blue_enu.y();
                double dist = std::sqrt(d_north * d_north + d_east * d_east);
                double u_n = (dist > 1.0) ? (d_north / dist) : 1.0;
                double u_e = (dist > 1.0) ? (d_east / dist) : 0.0;
                double perp_n = -u_e;
                double perp_e = u_n;
                double art_red_north = red_enu.x() + 200.0 * perp_n;
                double art_red_east = red_enu.y() + 200.0 * perp_e;
                enuToGps(art_red_north, art_red_east, art_red_lat, art_red_lon);

                double d_north_red = red_enu.x() - art_red_north;
                double d_east_red = red_enu.y() - art_red_east;
                double dist_red = std::sqrt(d_north_red * d_north_red + d_east_red * d_east_red);
                double u_n_red = (dist_red > 1.0) ? (d_north_red / dist_red) : 1.0;
                double u_e_red = (dist_red > 1.0) ? (d_east_red / dist_red) : 0.0;
                double new_red_north = red_enu.x() + 30.0 * u_n_red;
                double new_red_east = red_enu.y() + 30.0 * u_e_red;
                enuToGps(new_red_north, new_red_east, new_red_lat, new_red_lon);
            }

            RCLCPP_INFO(this->get_logger(), "[TELEMETRY C++] Handshake payload ready. Uploading 12 waypoints...");

            // Pass target coordinates to upload flight trajectory route pattern
            if (uploadCompleteMission(
                art_blue_lat, art_blue_lon, blue_lat, blue_lon, new_blue_lat, new_blue_lon,
                art_red_lat, art_red_lon, red_lat, red_lon, new_red_lat, new_red_lon
            )) {
                mission_plan_uploaded_ = true;
                RCLCPP_INFO(this->get_logger(), "[TELEMETRY C++] Flight corridor waypoints loaded successfully!");
            } else {
                RCLCPP_ERROR(this->get_logger(), "[TELEMETRY C++] Corridor waypoint upload failed. Retrying.");
            }
            uploading_mission_ = false;
        }).detach();
    }
}

bool MavlinkTelemetryDriver::uploadCompleteMission(
    double art_blue_lat, double art_blue_lon, double blue_lat, double blue_lon, double new_blue_lat, double new_blue_lon,
    double art_red_lat, double art_red_lon, double red_lat, double red_lon, double new_red_lat, double new_red_lon
) {
    std::unique_lock<std::mutex> lock(mission_mutex_);
    has_mission_request_ = false;
    has_mission_ack_ = false;

    mavlink_message_t msg;
    mavlink_msg_mission_count_pack(
        system_id_, component_id_, &msg,
        target_system_id_, target_component_id_,
        12, MAV_MISSION_TYPE_MISSION, 0
    );
    sendMavlinkMessage(msg);

    for (int i = 0; i < 12; i++) {
        bool success = mission_cv_.wait_for(lock, std::chrono::seconds(2), [this]() {
            return has_mission_request_;
        });

        if (!success) {
            RCLCPP_ERROR(this->get_logger(), "[MISSION HANDSHAKE] Handshake Timeout at item %d.", i);
            return false;
        }

        has_mission_request_ = false;
        uint16_t seq = requested_seq_;
        mavlink_message_t item_msg;

        if (seq == 0) {
            mavlink_msg_mission_item_int_pack(
                system_id_, component_id_, &item_msg, target_system_id_, target_component_id_,
                0, MAV_FRAME_MISSION, MAV_CMD_NAV_WAYPOINT, 0, 1, 0.0f, 0.0f, 0.0f, 0.0f,
                static_cast<int32_t>(home_lat_ * 1e7), static_cast<int32_t>(home_lon_ * 1e7), 0.0f, MAV_MISSION_TYPE_MISSION
            );
        } else if (seq == 1) {
            mavlink_msg_mission_item_int_pack(
                system_id_, component_id_, &item_msg, target_system_id_, target_component_id_,
                1, MAV_FRAME_GLOBAL_RELATIVE_ALT, MAV_CMD_NAV_WAYPOINT, 0, 1, 0.0f, 0.0f, 0.0f, 0.0f,
                static_cast<int32_t>(art_blue_lat * 1e7), static_cast<int32_t>(art_blue_lon * 1e7), 20.0f, MAV_MISSION_TYPE_MISSION
            );
        } else if (seq == 2) {
            mavlink_msg_mission_item_int_pack(
                system_id_, component_id_, &item_msg, target_system_id_, target_component_id_,
                2, MAV_FRAME_MISSION, MAV_CMD_DO_CHANGE_SPEED, 0, 1, 0.0f, 13.0f, -1.0f, 0.0f,
                0, 0, 0.0f, MAV_MISSION_TYPE_MISSION
            );
        } else if (seq == 3) {
            mavlink_msg_mission_item_int_pack(
                system_id_, component_id_, &item_msg, target_system_id_, target_component_id_,
                3, MAV_FRAME_GLOBAL_RELATIVE_ALT, MAV_CMD_NAV_WAYPOINT, 0, 1, 0.0f, 2.0f, 0.0f, 0.0f,
                static_cast<int32_t>(blue_lat * 1e7), static_cast<int32_t>(blue_lon * 1e7), 8.0f, MAV_MISSION_TYPE_MISSION
            );
        } else if (seq == 4) {
            mavlink_msg_mission_item_int_pack(
                system_id_, component_id_, &item_msg, target_system_id_, target_component_id_,
                4, MAV_FRAME_GLOBAL_RELATIVE_ALT, MAV_CMD_NAV_WAYPOINT, 0, 1, 0.0f, 0.0f, 0.0f, 0.0f,
                static_cast<int32_t>(new_blue_lat * 1e7), static_cast<int32_t>(new_blue_lon * 1e7), 8.0f, MAV_MISSION_TYPE_MISSION
            );
        } else if (seq == 5) {
            mavlink_msg_mission_item_int_pack(
                system_id_, component_id_, &item_msg, target_system_id_, target_component_id_,
                5, MAV_FRAME_MISSION, MAV_CMD_DO_CHANGE_SPEED, 0, 1, 0.0f, 13.0f, -1.0f, 0.0f,
                0, 0, 0.0f, MAV_MISSION_TYPE_MISSION
            );
        } else if (seq == 6) {
            mavlink_msg_mission_item_int_pack(
                system_id_, component_id_, &item_msg, target_system_id_, target_component_id_,
                6, MAV_FRAME_GLOBAL_RELATIVE_ALT, MAV_CMD_NAV_WAYPOINT, 0, 1, 0.0f, 0.0f, 0.0f, 0.0f,
                static_cast<int32_t>(art_red_lat * 1e7), static_cast<int32_t>(art_red_lon * 1e7), 20.0f, MAV_MISSION_TYPE_MISSION
            );
        } else if (seq == 7) {
            mavlink_msg_mission_item_int_pack(
                system_id_, component_id_, &item_msg, target_system_id_, target_component_id_,
                7, MAV_FRAME_MISSION, MAV_CMD_DO_CHANGE_SPEED, 0, 1, 0.0f, 13.0f, -1.0f, 0.0f,
                0, 0, 0.0f, MAV_MISSION_TYPE_MISSION
            );
        } else if (seq == 8) {
            mavlink_msg_mission_item_int_pack(
                system_id_, component_id_, &item_msg, target_system_id_, target_component_id_,
                8, MAV_FRAME_GLOBAL_RELATIVE_ALT, MAV_CMD_NAV_WAYPOINT, 0, 1, 0.0f, 2.0f, 0.0f, 0.0f,
                static_cast<int32_t>(red_lat * 1e7), static_cast<int32_t>(red_lon * 1e7), 8.0f, MAV_MISSION_TYPE_MISSION
            );
        } else if (seq == 9) {
            mavlink_msg_mission_item_int_pack(
                system_id_, component_id_, &item_msg, target_system_id_, target_component_id_,
                9, MAV_FRAME_GLOBAL_RELATIVE_ALT, MAV_CMD_NAV_WAYPOINT, 0, 1, 0.0f, 0.0f, 0.0f, 0.0f,
                static_cast<int32_t>(new_red_lat * 1e7), static_cast<int32_t>(new_red_lon * 1e7), 8.0f, MAV_MISSION_TYPE_MISSION
            );
        } else if (seq == 10) {
            mavlink_msg_mission_item_int_pack(
                system_id_, component_id_, &item_msg, target_system_id_, target_component_id_,
                10, MAV_FRAME_MISSION, MAV_CMD_DO_CHANGE_SPEED, 0, 1, 0.0f, 13.0f, -1.0f, 0.0f,
                0, 0, 0.0f, MAV_MISSION_TYPE_MISSION
            );
        } else if (seq == 11) {
            mavlink_msg_mission_item_int_pack(
                system_id_, component_id_, &item_msg, target_system_id_, target_component_id_,
                11, MAV_FRAME_GLOBAL_RELATIVE_ALT, MAV_CMD_NAV_WAYPOINT, 0, 1, 0.0f, 0.0f, 0.0f, 0.0f,
                static_cast<int32_t>(home_lat_ * 1e7), static_cast<int32_t>(home_lon_ * 1e7), 40.0f, MAV_MISSION_TYPE_MISSION
            );
        }

        sendMavlinkMessage(item_msg);
    }
    
    bool ack_received = mission_cv_.wait_for(lock, std::chrono::seconds(2), [this]() {
        return has_mission_ack_;
    });

    if (!ack_received) {
        RCLCPP_ERROR(this->get_logger(), "[MISSION HANDSHAKE] Timeout waiting for final MISSION_ACK.");
        return false;
    }

    has_mission_ack_ = false;

    if (ack_type_ == MAV_MISSION_ACCEPTED) {
        RCLCPP_INFO(this->get_logger(), "[MISSION HANDSHAKE] Mission corridor accepted by Autopilot.");
        
        mavlink_message_t active_wp_msg;
        mavlink_msg_mission_set_current_pack(
            system_id_, component_id_, &active_wp_msg, target_system_id_, target_component_id_, 1
        );
        sendMavlinkMessage(active_wp_msg);
        return true;
    } else {
        RCLCPP_ERROR(this->get_logger(), "[MISSION HANDSHAKE] Handshake REJECTED! Code: %d.", ack_type_);
        return false;
    }
}

void MavlinkTelemetryDriver::sendServoOutput(uint8_t servo_num, float pwm) {
    mavlink_message_t msg;
    mavlink_msg_command_long_pack(
        system_id_, component_id_, &msg,
        target_system_id_, target_component_id_,
        MAV_CMD_DO_SET_SERVO, 0,
        static_cast<float>(servo_num), pwm, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f
    );
    sendMavlinkMessage(msg);
}

void MavlinkTelemetryDriver::requestMessageInterval(uint32_t msg_id, int32_t interval_us) {
    mavlink_message_t msg;
    mavlink_msg_command_long_pack(
        system_id_, component_id_, &msg,
        target_system_id_, target_component_id_,
        MAV_CMD_SET_MESSAGE_INTERVAL, 0,
        static_cast<float>(msg_id), static_cast<float>(interval_us),
        0.0f, 0.0f, 0.0f, 0.0f, 0.0f
    );
    sendMavlinkMessage(msg);
}

void MavlinkTelemetryDriver::sendParameterSet(const char* param_name, float param_value, uint8_t param_type) {
    mavlink_message_t param_msg;
    char param_id[16] = {0};
    strncpy(param_id, param_name, sizeof(param_id) - 1);

    float value_to_send = 0.0f;
    if (param_type == MAV_PARAM_TYPE_INT16) {
        int16_t temp = static_cast<int16_t>(param_value);
        std::memcpy(&value_to_send, &temp, sizeof(temp));
    } else {
        value_to_send = param_value;
    }

    mavlink_msg_param_set_pack(
        system_id_, component_id_, &param_msg,
        target_system_id_, target_component_id_,
        param_id, value_to_send, param_type
    );
    sendMavlinkMessage(param_msg);
}

void MavlinkTelemetryDriver::requestDataStream(uint8_t stream_id, uint16_t rate) {
    if (socket_fd_ < 0 || !remote_addr_cached_) return;
    mavlink_message_t request_msg;
    mavlink_msg_request_data_stream_pack(
        system_id_, component_id_, &request_msg,
        target_system_id_, target_component_id_,
        stream_id, rate, 1
    );
    sendMavlinkMessage(request_msg);
}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MavlinkTelemetryDriver>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
