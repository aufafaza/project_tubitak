#include <Eigen/Dense>
#include <vector>
#include <cmath>
#include "point.hpp"
#include "ballistic_model.hpp"

/* Algorithm 1 and simulating equation 3 in paper */

/* Returns x dot vector [dx, dy, dz, dvx, dvy, dvz] using equation 3 */
StateVector getSDot(StateVector& s, PhysicsParams &p, double wx, double wy, double wz) {
    // Relative speed of payload velocity to wind velocity
    double rels_x = s(3) - wx;
    double rels_y = s(4) - wy;
    double rels_z = s(5) - wz;

    double Vr = sqrt(pow(rels_x, 2) + pow(rels_y, 2) + pow(rels_z, 2));
    double drag_param = (p.Cd * p.rho *p.A)/(2 * p.m);

    StateVector s_dot;
    s_dot(0) = s(3);
    s_dot(1) = s(4);
    s_dot(2) = s(5);
    s_dot(3) = -1 * drag_param * rels_x * Vr;
    s_dot(4) = -1 * drag_param * rels_y * Vr;
    s_dot(5) = p.g - (drag_param * rels_z * Vr);

    return s_dot;
}

/* Returns suggested release point based on ballistic model */
/* Using NED target coords, current UAV state, and current windspeed */
Point2D computeReleasePoint(
    double dt, double tgt_ned_x, double tgt_ned_y, double tgt_ned_z,
    double uav_z,
    double uav_vx, double uav_vy, double uav_vz,
    double wx, double wy, double wz) {

    
    // Assume drop starts at (0, 0, uav_z),
    // So that the final s(0) and s(1) is the distance the payload drifted
    // s is payload state
    StateVector s(
        0, 0, uav_z,
        uav_vx, uav_vy, uav_vz
    );

    while (s(2) < 0) {
        // Update wind estimate based on equation 5 and Touma (reference Z2 and W2 to uav state)
        double height_ratio_pow = pow(abs(s(2) / uav_z), 1.0/7.0);

        double new_wx = wx * height_ratio_pow;
        double new_wy = wy * height_ratio_pow;
        double new_wz = wz * height_ratio_pow;

        StateVector s_dot = getSDot(s, PHYS_PARAMS, new_wx, new_wy, new_wz);
        
        // Simulate equation (3) with dt
        s += s_dot * dt;
    }

    Point2D release_coord {(tgt_ned_x - s(0)), (tgt_ned_y - s(1))};

    return release_coord;
}