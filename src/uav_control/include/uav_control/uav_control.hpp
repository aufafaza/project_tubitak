#pragma once 
#include <mavsdk/mavsdk.h> 
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/mission/mission.h>
#include <mavsdk/plugins/mavlink_passthrough/mavlink_passthrough.h>
#include <mavsdk/plugins/param/param.h>
#include <mavsdk/plugins/telemetry/telemetry.h>

#include <atomic> 
#include <memory> 
#include <mutex> 
#include <optional> 
#include <string> 
#include <vector> 

class Mav{ 
public: 
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

    void sendServoCommand(int servo_number, float pwm_value);

    void getMissionState(MissionState& state);

    void setMode(const mavsdk::Mission::ResultCallback& callback, const std::string& mode); 

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

    // telemetry state 
    mutable std::mutex _mtx; 
    std::optional<GpsState> _gps; 
    std::optional<AttState> _att; 
    std::optional<VelState> _vel; 
    std::optional<NedState> _ned; 
    std::optional<HeadingState> _hdg; 
    std::atomic<bool> _gps_ok{false}; 
    std::atomic<bool> _connected{false}; 
};