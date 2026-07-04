#include <vector>
#include <cmath>
#include "ballistic_model.hpp"
#include "point.hpp"
#include "dubins_planner.hpp"

/* Airdrop Plan FSM based on paper */
/* Assumes constant uav height */

#define DROP_THRESHOLD_RADIUS 1.5 // allowed uav position deviation from drop coord in meters
#define STATE_TRANSITION_THRESHOLD_RADIUS 30 // allowed threshold for a change in state in meters
#define BALLISTIC_SIM_DT 0.1 // time step in ballistic simulation

// Moore FSM
enum class DropState {
    SEARCHING,          // wait for target detection
    RPATH_PLANNING,     // calculate rough release point estimate with airspeed, calculate and send path to release point opposing wind direction
    FLY_TO_RPATH,       // wait til it crosses release path
    RPOINT_PLANNING,    // calculate and send accurate release point with groundspeed
    FLY_TO_RPOINT,      // wait til uav pos is on release point within drop threshold
    RELEASING,          // send mavlink drop payload command
    COMPLETED           // finish payload drop mission
};

DropState current_state = DropState::SEARCHING;

void payloadStateMachine(
    bool is_target_visible, double tgt_ned_x, double tgt_ned_y, double tgt_ned_z,
    double uav_x, double uav_y, double uav_z,
    double uav_vx, double uav_vy, double uav_vz,
    double wx, double wy, double wz) {

    static Point2D release_point;
    static DubinsPath current_path;
    
    // Clear latch when not in drop state
    static bool has_entered_zone = false;
    if (current_state != DropState::FLY_TO_RPOINT) has_entered_zone = false;

    switch (current_state) {

        case DropState::SEARCHING: {
            if (is_target_visible) {
                current_state = DropState::RPATH_PLANNING;
            }
            break;
        }
        
        case DropState::RPATH_PLANNING: {
            // TODO: Generate approach line and loiter circle, send to ArduPilot
            
            // current_state = DropState::FLY_TO_RPATH;
            break;
        }

        case DropState::FLY_TO_RPATH: {
            double dist_to_p = sqrt(pow(uav_x - current_path.approach_start.x, 2) + pow(uav_y - current_path.approach_start.y, 2));

            double vec_to_p_x = current_path.approach_start.x - uav_x;
            double vec_to_p_y = current_path.approach_start.y - uav_y;
            double dot_product = (vec_to_p_x * uav_vx) + (vec_to_p_y * uav_vy);

            bool passed_p = (dot_product < 0.0);

            if (passed_p && dist_to_p < STATE_TRANSITION_THRESHOLD_RADIUS) {
                current_state = DropState::RPOINT_PLANNING;
            }
            
            break;
        }

        case DropState::RPOINT_PLANNING: {

            release_point = computeReleasePoint(
                BALLISTIC_SIM_DT,
                tgt_ned_x, tgt_ned_y, tgt_ned_z,
                uav_z,
                uav_vx, uav_vy, uav_vz,
                wx, wy, wz
            );
            // send mavlink message for intended release coord
            
            current_state = DropState::FLY_TO_RPOINT;
            break;
        }

        case DropState::FLY_TO_RPOINT: {
            // Drop when in drop radius and is moving away from release point. 

            double dist_to_drop = sqrt(pow(uav_x - release_point.x, 2) + pow(uav_y - release_point.y, 2));
            
            if (dist_to_drop <= DROP_THRESHOLD_RADIUS) {
                has_entered_zone = true;
            }

            double vec_to_tgt_x = release_point.x - uav_x;
            double vec_to_tgt_y = release_point.y - uav_y;
            double dot_product = (vec_to_tgt_x * uav_vx) + (vec_to_tgt_y * uav_vy);

            bool is_moving_away = (dot_product < 0.0);

            if (is_moving_away && dist_to_drop < STATE_TRANSITION_THRESHOLD_RADIUS) {
                if (has_entered_zone) current_state = DropState::RELEASING;
                else current_state = DropState::RPATH_PLANNING;
            }

            break;
        }

        case DropState::RELEASING: {
            // send mavlink payload servo command

            current_state = DropState::COMPLETED;
            break;
        }

        case DropState::COMPLETED:
            // send mavlink command to auto/loiter/etc

            break;
    }
}


