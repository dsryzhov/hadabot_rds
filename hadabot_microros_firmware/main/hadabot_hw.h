#ifndef __HADABOT_HW_H
#define __HADABOT_HW_H

#include "motor.h"
#include "rotsensor.h"



class HadabotHW {
public:
	HadabotHW();
	~HadabotHW();
	
	void init();
	
	inline Motor* getLeftMotor() {return &leftMotor; }
	inline Motor* getRightMotor() {return &rightMotor; }
	inline RotationSensor* getLeftWheelRotationSensor() {return &leftWheelRotationSensor;}	
	inline RotationSensor* getRightWheelRotationSensor() {return &rightWheelRotationSensor;}	
	
protected:
	Motor leftMotor;
	Motor rightMotor;
	RotationSensor leftWheelRotationSensor;
	RotationSensor rightWheelRotationSensor;
	
};

#endif