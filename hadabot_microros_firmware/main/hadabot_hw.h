#ifndef __HADABOT_HW_H
#define __HADABOT_HW_H

#include "motor.h"
#include "rotsensor.h"
#include "hcsr04.h"
#include "PosEstimator.h"
//#include "TinyMPU6050.h"
#include "MPU6050.h"


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
	inline PosEstimator* getPosEstimator() { return &pos_estimator;}
	inline MPU6050* getMPU6050() {return &mpu;}
	
	void updatePosition();
	
protected:
	Motor leftMotor;
	Motor rightMotor;
	RotationSensor leftWheelRotationSensor;
	RotationSensor rightWheelRotationSensor;
	HCSR04 hcsr04;
	MPU6050 mpu;
	
	PosEstimator pos_estimator;

};

#endif