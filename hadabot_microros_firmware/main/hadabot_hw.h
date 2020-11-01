#ifndef __HADABOT_HW_H
#define __HADABOT_HW_H

#include "motor.h"
#include "rotsensor.h"
#include "hcsr04.h"
#include "PosEstimator.h"


class HadabotHW {
public:
	HadabotHW();
	~HadabotHW();
	
	void begin();
	
	inline Motor* getLeftMotor() {return &leftMotor; }
	inline Motor* getRightMotor() {return &rightMotor; }
	inline RotationSensor* getLeftWheelRotationSensor() {return &leftWheelRotationSensor;}	
	inline RotationSensor* getRightWheelRotationSensor() {return &rightWheelRotationSensor;}	
	inline HCSR04* getHCSR04() {return &hcsr04;}

	void updatePosition();
	
protected:
	Motor leftMotor;
	Motor rightMotor;
	RotationSensor leftWheelRotationSensor;
	RotationSensor rightWheelRotationSensor;
	HCSR04 hcsr04;
	PosEstimator pos_estimator;

};

#endif