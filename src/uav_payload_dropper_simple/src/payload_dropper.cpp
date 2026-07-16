#include "uav_payload_dropper_simple/payload_dropper.hpp"
#include <mavsdk/plugins/action/action.h>
#include <cmath>
#include <iostream>

Dropper::Dropper(std::shared_ptr<mavsdk::System> system)
    : system_(std::move(system)) {}

void Dropper::setState(double vx_ms, double vy_ms, double altitude_m)
{
    vx_ = vx_ms;
    vy_ = vy_ms;
    altitude_ = altitude_m;
}

double Dropper::dropDistance() const
{
    if (altitude_ <= 0.0) return 0.0;
    double speed = std::sqrt(vx_ * vx_ + vy_ * vy_);
    double fall_time = std::sqrt(2.0 * altitude_ / kGravity);
    return speed * fall_time;
}

bool Dropper::shouldDrop(double dist_to_target_m) const
{
    return dist_to_target_m <= dropDistance();
}

void Dropper::drop(int32_t servo_index, float open_pwm)
{
    setServoPWM(servo_index, open_pwm);
}

void Dropper::setServoPWM(int32_t index, float pwm_value)
{
    auto action = mavsdk::Action{system_};
    action.set_actuator_async(index, pwm_value, [](mavsdk::Action::Result result) {
        if (result == mavsdk::Action::Result::Success)
            std::cout << "Payload released\n";
        else
            std::cout << "Servo command failed\n";
    });
}
