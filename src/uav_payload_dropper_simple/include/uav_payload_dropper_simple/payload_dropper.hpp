#pragma once 

#include <mavsdk/mavsdk.h> 

class Dropper { 
public: 
	Dropper(std::shared_ptr<mavsdk::System> system); 
	void setServoPWM(int index, float pwm_value);
	void calculateDropDistance(); 	
	//TODO: calculate drop distance should check whether the heading of the vechile relative to the waypoint is perpendicular, if yes, calculate at the posed timestamp its altitude, vx, vy. After that, do simple physics calculation
private: 
	//constants 
	std::shared_ptr<mavsdk::System> system; 
	const double gravity = 9.8; 
	double vx, vy, altitude; 	
}; 
