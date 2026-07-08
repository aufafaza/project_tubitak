#pragma once 

#include "point.hpp"
#include "dubins_planner.hpp"
#include "ballistic_model.hpp"

// Moore FSM States
enum class DropState {
    SEARCHING,          
    RPATH_PLANNING,     
    FLY_TO_RPATH,       
    RPOINT_PLANNING,    
    FLY_TO_RPOINT,      
    RELEASING,          
    COMPLETED           
};

class AirdropManager {
private:
    // Persistent state variables
    DropState current_state;
    Point2D release_point;
    DubinsPath current_path;
    bool has_entered_zone;

    // Tuning Constants
    static constexpr double DROP_THRESHOLD_RADIUS = 1.5; 
    static constexpr double STATE_TRANSITION_THRESHOLD_RADIUS = 30.0; 
    static constexpr double BALLISTIC_SIM_DT = 0.1; 
    static constexpr double LOITER_RADIUS = 30.0; 
    static constexpr double APPROACH_DIST = 100.0; 

public:
    // Constructor
    AirdropManager();

    // Safely resets the state machine for a new airdrop mission
    void resetMission();

    // Returns the current state
    DropState getCurrentState() const;

    // Main update loop
    void update(
        bool is_target_visible, double tgt_ned_x, double tgt_ned_y, double tgt_ned_z,
        double uav_x, double uav_y, double uav_z,
        double uav_vx, double uav_vy, double uav_vz,
        double wx, double wy, double wz
    );
};
