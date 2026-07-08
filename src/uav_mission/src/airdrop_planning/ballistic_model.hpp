#include <Eigen/Dense>
#include <vector>
#include <cmath>
#include "point.hpp"

#pragma once 
/* Algorithm 1 and simulating equation 3 in paper */

/* x vector [x, y, z, vx, vy, vz] */
typedef Eigen::Matrix<double, 6, 1> StateVector;

/* Payload physics parameters */
typedef struct PhysicsParams {
    double Cd, rho, A, m;
    double g = 9.81;
} PhysicsParams;

static PhysicsParams PHYS_PARAMS = {1, 1.25, 4, 5, 9.81}; // Update according to actual vals

/* Returns x dot vector [dx, dy, dz, dvx, dvy, dvz] using equation 3 */
StateVector getSDot(StateVector& s, PhysicsParams &p, double wx, double wy, double wz);

/* Returns suggested release point based on ballistic model */
/* Using NED target coords, current UAV state, and current windspeed */
Point2D computeReleasePoint(
    double dt, double tgt_ned_x, double tgt_ned_y, double tgt_ned_z,
    double uav_z,
    double uav_vx, double uav_vy, double uav_vz,
    double wx, double wy, double wz);
