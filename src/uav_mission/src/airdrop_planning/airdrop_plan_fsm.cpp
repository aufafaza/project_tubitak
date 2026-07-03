#include <vector>
#include <cmath>
#include "ballistic_model.hpp"
#include "point.hpp"

/* Airdrop Plan FSM based on paper */
/* Assumes constant uav height */

#define DROP_THRESHOLD_RADIUS 1.5 // allowed uav position deviation from drop coord in meters
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

    Point2D release_point;

    switch (current_state) {

        case DropState::SEARCHING: {
            if (is_target_visible) {
                current_state = DropState::RPOINT_PLANNING;
            }
            break;
        }
        
        case DropState::RPATH_PLANNING: {
            // todo later
            break;
        }

        case DropState::FLY_TO_RPATH: {
            // todo later
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
        }

        case DropState::FLY_TO_RPOINT: {
            double dist_to_drop = sqrt(pow(uav_x - release_point.x, 2) + pow(uav_y - release_point.y, 2));
            
            if (dist_to_drop <= DROP_THRESHOLD_RADIUS) current_state = DropState::RELEASING;
            break;
        }

        case DropState::RELEASING: {
            // send mavlink payload servo command

            current_state = DropState::COMPLETED;
            break;
        }

        case DropState::COMPLETED:
            // complete mission code

            break;
    }
}


