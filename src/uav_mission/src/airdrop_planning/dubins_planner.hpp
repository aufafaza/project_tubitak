#ifndef DUBINS_PLANNER_HPP
#define DUBINS_PLANNER_HPP

#include "point.hpp"

struct DubinsPath {
    Point2D release_point;  // Final drop coordinate
    Point2D approach_start; // Point 'p' (start of final approach line)
    Point2D loiter_center;  // Point 's' (center of tangent circle)
    double loiter_radius;   // Radius 'r'
};

/* * Generates the approach path to fly into the wind, 
 * complete a clockwise loiter, and line up for the drop.
 */
DubinsPath generateApproachPath(
    Point2D release_point, 
    double wx, double wy, 
    double approach_dist_d, 
    double loiter_radius_r,
    double current_uav_vx, double current_uav_vy // Fallback if wind is zero
);

#endif