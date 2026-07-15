#include "uav_payload_dropper_simple/payload_dropper.hpp" 
#include <mavsdk/mavsdk.h> 
#include <mavsdk/plugins/action/action.h>
#include <iostream> 

// class Dropper { 
// public: 
// 	Dropper(std::shared_ptr<mavsdk::System> system); 
// 	void set_servo_pwm(int index, float pwm_value);
// private: 
// 	//constants 
// 	const double gravity = 9.8; 
// 	double vx, vy, altitude; 	
// }; 
//
Dropper::Dropper(std::shared_ptr<mavsdk::System> system) { 
	this->system = std::move(system); 
} 

void Dropper::setServoPWM(int32_t index, float pwm_value){ 
	auto action = mavsdk::Action{this->system}; 
	action.set_actuator_async(index, pwm_value, [](mavsdk::Action::Result result){
		if (result == mavsdk::Action::Result::Success){ 
			std::cout << "success" << std::endl;
		}else{ 
			   std:: cout << "fail" << std::endl;
		} 
	});
} 
