#include "dubins_planner.hpp"
#include <cmath>

DubinsPath generateApproachPath(
    Point2D tgt_ned, 
    double wx, double wy, 
    double approach_dist_d, 
    double loiter_radius_r,
    double current_uav_vx, double current_uav_vy,
    double current_uav_x, double current_uav_y,
    bool &is_cw) {

    DubinsPath path;
    path.tgt_ned = tgt_ned;
    path.loiter_radius = loiter_radius_r;

    // Calculate approach unit vector
    double wind_mag = std::sqrt(wx * wx + wy * wy);
    double u_wx, u_wy;
    
    if (wind_mag > MIN_DEFINED_SPEED) { 
        u_wx = -1.0 * (wx / wind_mag);
        u_wy = -1.0 * (wy / wind_mag);
    } else { 
        double uav_v_mag = std::sqrt(current_uav_vx * current_uav_vx + current_uav_vy * current_uav_vy);
        if (uav_v_mag > MIN_DEFINED_SPEED) {
            u_wx = current_uav_vx / uav_v_mag;
            u_wy = current_uav_vy / uav_v_mag;
        } else { 
            u_wx = 1.0;
            u_wy = 0.0;
        }
    }

    // Calculate 'p' (start of approach line)
    path.approach_start.x = tgt_ned.x - (approach_dist_d * u_wx);
    path.approach_start.y = tgt_ned.y - (approach_dist_d * u_wy);

    // Determine cw or ccw loiter based on current position relative to p
    double vec_uav_p_x = current_uav_x - path.approach_start.x;
    double vec_uav_p_y = current_uav_y - path.approach_start.y;
    double cross_product = (u_wx * vec_uav_p_y) - (u_wy * vec_uav_p_x);

    double vec_s_offset_x, vec_s_offset_y;
    if (cross_product > 0) {
        // CCW (Left)
        vec_s_offset_x = u_wy;
        vec_s_offset_y = -u_wx;
        is_cw = false;
    } else {
        // CW (Right)
        vec_s_offset_x = -u_wy;
        vec_s_offset_y = u_wx;
        is_cw = true;
    }

    // Calculate 's' (center of loiter circle)
    path.loiter_center.x = path.approach_start.x + (loiter_radius_r * vec_s_offset_x);
    path.loiter_center.y = path.approach_start.y + (loiter_radius_r * vec_s_offset_y);

    return path;
}