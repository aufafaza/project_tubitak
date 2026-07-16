#pragma once

#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/action/action.h>

class Dropper {
public:
    Dropper(std::shared_ptr<mavsdk::System> system);

    // Update current drone state before calling shouldDrop / drop
    void setState(double vx_ms, double vy_ms, double altitude_m);

    // Horizontal distance (m) payload travels before hitting ground: speed * sqrt(2h/g)
    double dropDistance() const;

    // True when drone is within drop distance of the target
    bool shouldDrop(double dist_to_target_m) const;

    // Releases payload via servo
    void drop(int32_t servo_index = 6, float open_pwm = 1900.0f);

private:
    void setServoPWM(int32_t index, float pwm_value);

    std::shared_ptr<mavsdk::System> system_;
    static constexpr double kGravity = 9.81;
    double vx_{0.0};
    double vy_{0.0};
    double altitude_{0.0};
};
