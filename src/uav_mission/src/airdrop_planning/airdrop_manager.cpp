#include "airdrop_manager.hpp"
#include <cmath>

AirdropManager::AirdropManager() {
    resetMission();
}

void AirdropManager::resetMission() {
    current_state = DropState::SEARCHING;
    has_entered_zone = false;
}

DropState AirdropManager::getCurrentState() const {
    return current_state;
}

void AirdropManager::update(
    bool is_target_visible, double tgt_ned_x, double tgt_ned_y, double tgt_ned_z,
    double uav_x, double uav_y, double uav_z,
    double uav_vx, double uav_vy, double uav_vz,
    double wx, double wy, double wz) {

    // Clear latch when not in drop state
    if (current_state != DropState::FLY_TO_RPOINT) {
        has_entered_zone = false;
    }

    switch (current_state) {

        case DropState::SEARCHING: {
            if (is_target_visible) {
                current_state = DropState::RPATH_PLANNING;
            }
            break;
        }
        
        case DropState::RPATH_PLANNING: {
            Point2D target_pt {tgt_ned_x, tgt_ned_y};
            bool is_cw = true;

            current_path = generateApproachPath(
                target_pt, wx, wy,
                APPROACH_DIST, LOITER_RADIUS,
                uav_vx, uav_vy,
                uav_x, uav_y,
                is_cw
            );
            
            double commanded_radius = is_cw ? LOITER_RADIUS : -LOITER_RADIUS;
            
            // TODO: MAVLink command - Loiter Unlimited at current_path.loiter_center with commanded_radius
            
            current_state = DropState::FLY_TO_RPATH;
            break;
        }

        case DropState::FLY_TO_RPATH: {
            double dist_to_p = std::sqrt(std::pow(uav_x - current_path.approach_start.x, 2) + 
                                         std::pow(uav_y - current_path.approach_start.y, 2));

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
                uav_z, uav_vx, uav_vy, uav_vz,
                wx, wy, wz
            );
            
            // TODO: MAVLink command - Fly to Waypoint at release_point
            
            current_state = DropState::FLY_TO_RPOINT;
            break;
        }

        case DropState::FLY_TO_RPOINT: {
            double dist_to_drop = std::sqrt(std::pow(uav_x - release_point.x, 2) + 
                                            std::pow(uav_y - release_point.y, 2));
            
            if (dist_to_drop <= DROP_THRESHOLD_RADIUS) {
                has_entered_zone = true;
            }

            double vec_to_tgt_x = release_point.x - uav_x;
            double vec_to_tgt_y = release_point.y - uav_y;
            double dot_product = (vec_to_tgt_x * uav_vx) + (vec_to_tgt_y * uav_vy);

            bool is_moving_away = (dot_product < 0.0);

            if (is_moving_away && dist_to_drop < STATE_TRANSITION_THRESHOLD_RADIUS) {
                if (has_entered_zone) {
                    current_state = DropState::RELEASING;
                } else {
                    current_state = DropState::RPATH_PLANNING;
                }
            }
            break;
        }

        case DropState::RELEASING: {
            // TODO: MAVLink command - Trigger payload servo
            
            current_state = DropState::COMPLETED;
            break;
        }

        case DropState::COMPLETED: {
            // TODO: MAVLink command - RTL or AUTO
            // Safely idles here until AirdropManager::resetMission() is called
            break;
        }
    }
}