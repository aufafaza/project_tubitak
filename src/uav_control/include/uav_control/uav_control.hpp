#pragma once 
#include <mavsdk/mavsdk.h> 
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/mission/mission.h>
#include <mavsdk/plugins/mavlink_passthrough/mavlink_passthrough.h>
#include <mavsdk/plugins/param/param.h>
#include <mavsdk/plugins/telemetry/telemetry.h>

#include <atomic>
#include <chrono>
#include <deque>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

class Mav{
public:
    using TimePoint = int64_t; // nanoseconds, steady_clock

    template<typename T>
    struct Stamped { TimePoint t; T v; };

    Mav(std::string connectionstring);
    ~Mav() = default;
    enum class MissionState { 
        NONE = -1, 
        IDLE = 0, 
        TAKEOFF = 1, 
        CRUISE = 2, 
        DROP = 3 
    }; 

    struct AttState{ 
        float roll_rad{0}, pitch_rad{0}, yaw_rad{0}; 
    };

    struct GpsState{ 
        double lat{0}, lon{0}; 
        float alt_msl{0}, alt_rel{0};
        double hdg_deg{0}; 
    };

    struct VelState{ 
        float north_m_s{0}, east_m_s{0}, down_m_s{0}; 
    }; 

    struct NedState{ 
        float north{0}, east{0}, down{0};  
    };
    
    struct HeadingState{ 
        double heading_deg{0}; 
    };
    
    void createWaypoint(const mavsdk::Mission::MissionItem& item);
    void updateLastWaypoint(const mavsdk::Mission::MissionItem& item);

    // Download current ArduPilot mission, append item, re-upload (serialized by internal mutex)
    void appendLiveWaypoint(const mavsdk::Mission::MissionItem& item,
                            std::function<void(bool)> callback = nullptr);

    void sendServoCommand(int servo_number, float pwm_value);

    void getMissionState(MissionState& state);

    void setMode(const mavsdk::Mission::ResultCallback& callback, const std::string& mode);

    // Latest cached telemetry (non-blocking)
    std::optional<GpsState> getGPS() const;
    std::optional<AttState> getAtt() const;
    std::optional<NedState> getPositionNed() const;
    std::optional<VelState> getVelocityNed() const;
    bool gpsSafeCheck() const;

    // Interpolated pose at a specific capture timestamp — use for georeferencing
    std::optional<AttState> getAttAt(TimePoint t) const;
    std::optional<NedState> getNedAt(TimePoint t) const;

private: 
    void _startSubscription(); 
    mavsdk::Mavsdk _mavsdk; 
    std::shared_ptr<mavsdk::System> _system;    
    std::unique_ptr<mavsdk::Telemetry> _telemetry; 
    std::unique_ptr<mavsdk::Action> _action;
    std::unique_ptr<mavsdk::MavlinkPassthrough> _passthrough;
    std::vector<mavsdk::Mission::MissionItem> _mission;
    std::unique_ptr<mavsdk::Param> _param;

    // subscription 
    mavsdk::Telemetry::PositionHandle _h_pos; 
    mavsdk::Telemetry::AttitudeEulerHandle _h_att; 
    mavsdk::Telemetry::PositionVelocityNedHandle _h_pos_vel; 
    mavsdk::Telemetry::GpsInfoHandle _h_pos_info; 
    mavsdk::Telemetry::HeadingHandle _h_hdg; 

    // telemetry state (latest values)
    mutable std::mutex _mtx;
    std::mutex _mission_mtx;  // serializes mission download/upload operations
    std::optional<GpsState>     _gps;
    std::optional<AttState>     _att;
    std::optional<VelState>     _vel;
    std::optional<NedState>     _ned;
    std::optional<HeadingState> _hdg;
    std::atomic<bool> _gps_ok{false};
    std::atomic<bool> _connected{false};

    // timestamped history for interpolation (~3s at 20 Hz)
    static constexpr size_t TELEM_BUF = 64;
    std::deque<Stamped<AttState>> _att_buf;
    std::deque<Stamped<NedState>> _ned_buf;
};