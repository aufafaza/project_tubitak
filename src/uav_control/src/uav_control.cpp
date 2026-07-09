#include "uav_control/uav_control.hpp"
#include <mavsdk/plugins/mission/mission.h>
#include <algorithm>
#include <stdexcept>

// ArduPlane custom mode numbers (MAV_CMD_DO_SET_MODE param2)
static constexpr uint8_t ARDU_MODE_FBWA   = 5;
static constexpr uint8_t ARDU_MODE_AUTO   = 10;
static constexpr uint8_t ARDU_MODE_LOITER = 12;

Mav::Mav(std::string connectionstring)
    : _mavsdk(mavsdk::Mavsdk::Configuration{mavsdk::ComponentType::GroundStation})
{
    _mavsdk.add_any_connection(connectionstring);
    auto system = _mavsdk.first_autopilot(10.0);
    if (!system){
        throw std::runtime_error("No AP found");
    }
    _system = *system;
    _telemetry    = std::make_unique<mavsdk::Telemetry>(_system);
    _action       = std::make_unique<mavsdk::Action>(_system);
    _param        = std::make_unique<mavsdk::Param>(_system);
    _passthrough  = std::make_unique<mavsdk::MavlinkPassthrough>(_system);
    _startSubscription();
    _connected = true;
}

void Mav::_startSubscription(){
    _telemetry->set_rate_position(10.0);
    _telemetry->set_rate_attitude_euler(20.0);
    _telemetry->set_rate_position_velocity_ned(10.0);

    _h_pos = _telemetry->subscribe_position([this](mavsdk::Telemetry::Position pos){
        std::lock_guard<std::mutex> lock(_mtx);
        _gps = GpsState{
            pos.latitude_deg, pos.longitude_deg,
            pos.absolute_altitude_m, pos.relative_altitude_m
        };
    });

    _h_att = _telemetry->subscribe_attitude_euler([this](mavsdk::Telemetry::EulerAngle ea){
        constexpr float deg2rad = M_PI / 180.0f;
        AttState s{ea.roll_deg * deg2rad, ea.pitch_deg * deg2rad, ea.yaw_deg * deg2rad};
        std::lock_guard<std::mutex> lock(_mtx);
        _att = s;
        _att_buf.push_back({std::chrono::steady_clock::now(), s});
        if (_att_buf.size() > TELEM_BUF) _att_buf.pop_front();
    });

    _h_pos_vel = _telemetry->subscribe_position_velocity_ned([this](mavsdk::Telemetry::PositionVelocityNed posvel){
        NedState ned{posvel.position.north_m, posvel.position.east_m, posvel.position.down_m};
        std::lock_guard<std::mutex> lock(_mtx);
        _vel = VelState{posvel.velocity.north_m_s, posvel.velocity.east_m_s, posvel.velocity.down_m_s};
        _ned = ned;
        _ned_buf.push_back({std::chrono::steady_clock::now(), ned});
        if (_ned_buf.size() > TELEM_BUF) _ned_buf.pop_front();
    });

    _h_pos_info = _telemetry->subscribe_gps_info([this](mavsdk::Telemetry::GpsInfo gps_info){
        _gps_ok = (gps_info.fix_type == mavsdk::Telemetry::FixType::Fix3D
                   && gps_info.num_satellites >= 3);
    });

    _h_hdg = _telemetry->subscribe_heading([this](mavsdk::Telemetry::Heading hdg){
        std::lock_guard<std::mutex> lock(_mtx);
        _hdg = HeadingState{hdg.heading_deg};
        if (_gps) _gps->hdg_deg = hdg.heading_deg;
    });
}

static float lerpAngle(float a, float b, float alpha) {
    float diff = b - a;
    if (diff >  M_PI) diff -= 2.0f * M_PI;
    if (diff < -M_PI) diff += 2.0f * M_PI;
    return a + diff * alpha;
}

std::optional<Mav::AttState> Mav::getAttAt(TimePoint t) const {
    std::lock_guard<std::mutex> lock(_mtx);
    if (_att_buf.size() < 2) return _att;

    // Past all data, return oldest
    if (t <= _att_buf.front().t) return _att_buf.front().v;
    // Beyond latest, return newest
    if (t >= _att_buf.back().t)  return _att_buf.back().v;

    // Find bracketing pair
    for (size_t i = 0; i + 1 < _att_buf.size(); ++i) {
        if (_att_buf[i].t <= t && t <= _att_buf[i + 1].t) {
            float span  = std::chrono::duration<float>(_att_buf[i+1].t - _att_buf[i].t).count();
            float alpha = std::chrono::duration<float>(t - _att_buf[i].t).count() / span;
            const auto& a = _att_buf[i].v;
            const auto& b = _att_buf[i + 1].v;
            return AttState{
                lerpAngle(a.roll_rad,  b.roll_rad,  alpha),
                lerpAngle(a.pitch_rad, b.pitch_rad, alpha),
                lerpAngle(a.yaw_rad,   b.yaw_rad,   alpha)
            };
        }
    }
    return _att;
}

std::optional<Mav::NedState> Mav::getNedAt(TimePoint t) const {
    std::lock_guard<std::mutex> lock(_mtx);
    if (_ned_buf.size() < 2) return _ned;

    if (t <= _ned_buf.front().t) return _ned_buf.front().v;
    if (t >= _ned_buf.back().t)  return _ned_buf.back().v;

    for (size_t i = 0; i + 1 < _ned_buf.size(); ++i) {
        if (_ned_buf[i].t <= t && t <= _ned_buf[i + 1].t) {
            float span  = std::chrono::duration<float>(_ned_buf[i+1].t - _ned_buf[i].t).count();
            float alpha = std::chrono::duration<float>(t - _ned_buf[i].t).count() / span;
            const auto& a = _ned_buf[i].v;
            const auto& b = _ned_buf[i + 1].v;
            return NedState{
                a.north + (b.north - a.north) * alpha,
                a.east  + (b.east  - a.east)  * alpha,
                a.down  + (b.down  - a.down)  * alpha
            };
        }
    }
    return _ned;
}

std::optional<Mav::GpsState> Mav::getGPS() const {
    std::lock_guard<std::mutex> lock(_mtx);
    return _gps;
}

std::optional<Mav::AttState> Mav::getAtt() const {
    std::lock_guard<std::mutex> lock(_mtx);
    return _att;
}

std::optional<Mav::NedState> Mav::getPositionNed() const {
    std::lock_guard<std::mutex> lock(_mtx);
    return _ned;
}

std::optional<Mav::VelState> Mav::getVelocityNed() const {
    std::lock_guard<std::mutex> lock(_mtx);
    return _vel;
}

bool Mav::gpsSafeCheck() const {
    return _gps_ok.load();
}

void Mav::createWaypoint(const mavsdk::Mission::MissionItem& item) {
    _mission.push_back(item);
}

void Mav::sendServoCommand(int servo_number, float pwm_value) {
    float normalized = (pwm_value - 1500.0f) / 500.0f;
    normalized = std::clamp(normalized, -1.0f, 1.0f);
    _action->set_actuator(servo_number, normalized);
}

void Mav::getMissionState(MissionState& state) {
    if (!_connected) {
        state = MissionState::NONE;
        return;
    }
    switch (_telemetry->flight_mode()) {
        case mavsdk::Telemetry::FlightMode::Takeoff:
            state = MissionState::TAKEOFF; break;
        case mavsdk::Telemetry::FlightMode::Mission:
            state = MissionState::CRUISE; break;
        case mavsdk::Telemetry::FlightMode::Hold:
        case mavsdk::Telemetry::FlightMode::Ready:
        case mavsdk::Telemetry::FlightMode::Land:
        case mavsdk::Telemetry::FlightMode::ReturnToLaunch:
            state = MissionState::IDLE; break;
        default:
            state = MissionState::NONE; break;
    }
}

void Mav::setMode(const mavsdk::Mission::ResultCallback& callback, const std::string& mode) {
    auto send_ardu_mode = [this](uint8_t custom_mode) -> bool {
        mavsdk::MavlinkPassthrough::CommandLong cmd{};
        cmd.target_sysid  = _passthrough->get_target_sysid();
        cmd.target_compid = _passthrough->get_target_compid();
        cmd.command       = 176; // MAV_CMD_DO_SET_MODE
        cmd.param1        = 1;   // MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
        cmd.param2        = static_cast<float>(custom_mode);
        return _passthrough->send_command_long(cmd)
               == mavsdk::MavlinkPassthrough::Result::Success;
    };

    auto ardu_result = [&callback](bool ok) {
        callback(ok ? mavsdk::Mission::Result::Success : mavsdk::Mission::Result::Error);
    };

    if (mode == "AUTO") {
        ardu_result(send_ardu_mode(ARDU_MODE_AUTO));
    } else if (mode == "FBWA") {
        ardu_result(send_ardu_mode(ARDU_MODE_FBWA));
    } else if (mode == "LOITER") {
        ardu_result(send_ardu_mode(ARDU_MODE_LOITER));
    } else if (mode == "RTL") {
        _action->return_to_launch_async([callback](mavsdk::Action::Result r) {
            callback(r == mavsdk::Action::Result::Success
                     ? mavsdk::Mission::Result::Success
                     : mavsdk::Mission::Result::Error);
        });
    } else if (mode == "START") {
        auto mp = std::make_shared<mavsdk::Mission>(_system);
        mavsdk::Mission::MissionPlan plan;
        plan.mission_items = _mission;
        mp->upload_mission_async(plan, [mp, callback](mavsdk::Mission::Result result) {
            if (result == mavsdk::Mission::Result::Success) {
                mp->start_mission_async(callback);
            } else {
                callback(result);
            }
        });
    } else if (mode == "PAUSE") {
        auto mp = std::make_shared<mavsdk::Mission>(_system);
        mp->pause_mission_async([mp, callback](mavsdk::Mission::Result r) { callback(r); });
    } else if (mode == "CLEAR") {
        _mission.clear();
        auto mp = std::make_shared<mavsdk::Mission>(_system);
        mp->clear_mission_async([mp, callback](mavsdk::Mission::Result r) { callback(r); });
    } else {
        callback(mavsdk::Mission::Result::Unknown);
    }
}
