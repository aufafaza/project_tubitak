#include "uav_control/uav_control.hpp"
#include <mavsdk/plugins/telemetry/telemetry.h>
Mav::Mav(std::string connectionstring)
    : _mavsdk(mavsdk::Mavsdk::Configuration{mavsdk::ComponentType::GroundStation})
{
    _mavsdk.add_any_connection(connectionstring);
    auto system = _mavsdk.first_autopilot(10.0);
    if (!system){ 
        throw std::runtime_error("No AP found"); 
    } 
    _system = *system; 
    _telemetry = std::make_unique<mavsdk::Telemetry>(_system); 
    _action = std::make_unique<mavsdk::Action>(_system); 
    _param = std::make_unique<mavsdk::Param>(_system); 
    this->_startSubscription();
}

void Mav::_startSubscription(){ 
    // setup subscription here 
    // to subscribe: position, att euler, pos vel, gps info, heading  

    // position 
    const mavsdk::Telemetry::Result set_rate_result = _telemetry->set_rate_position(1.0); 
    if (set_rate_result != mavsdk::Telemetry::Result::Success){ 
        throw std::runtime_error("Setting rate failed"); 
    }

    _h_pos = _telemetry->subscribe_position([this](mavsdk::Telemetry::Position pos){ 
        std::lock_guard<std::mutex> lock(_mtx); 
        _gps = GpsState{ 
                pos.latitude_deg, pos.longitude_deg, pos.absolute_altitude_m, pos.relative_altitude_m
        };
    });
    
    _h_att = _telemetry->subscribe_attitude_euler([this] (mavsdk::Telemetry::EulerAngle ea){ 
        constexpr float deg2rad = M_PI / 180.0; 
        std::lock_guard<std::mutex> lock(_mtx); 
        _att = AttState { 
           ea.roll_deg * deg2rad, ea.pitch_deg * deg2rad, ea.yaw_deg * deg2rad 
        }; 
    }); 

    _h_pos_vel = _telemetry->subscribe_position_velocity_ned([this] (mavsdk::Telemetry::PositionVelocityNed posvel){
        std::lock_guard<std::mutex> lock(_mtx); 
        _vel = VelState{ 
           posvel.velocity.north_m_s, posvel.velocity.east_m_s, posvel.velocity.down_m_s 
        };
        _ned = NedState{ 
            posvel.position.north_m, posvel.position.east_m, posvel.position.down_m
        };
    });

    _h_pos_info = _telemetry->subscribe_gps_info([this](mavsdk::Telemetry::GpsInfo gps_info){ 
        std::lock_guard<std::mutex> lock(_mtx); 
        // change to RTKFix later down the line  
        _gps_ok = (gps_info.fix_type == mavsdk::Telemetry::FixType::Fix3D && gps_info.num_satellites >= 3); 
    });

    _h_hdg = _telemetry->subscribe_heading([this] (mavsdk::Telemetry::Heading hdg){ 
        std::lock_guard<std::mutex> lock(_mtx); 
        _hdg = {
            hdg.heading_deg
        };   
    }); 

}