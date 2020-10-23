#ifndef __MOTOR_H
#define __MOTOR_H

#include <cstdint>

enum EMotorState {STOPED = 0, FORWARD = 1, BACKWARD = -1, STOPING_FORWARD = 2, STOPING_BACKWARD = -2};

class Motor {
public:
	Motor(uint8_t _forward_channel, uint8_t _backward_channel, uint8_t forward_pin_num, uint8_t backward_pin_num, double freq=1000);
	~Motor(); 
	void forward(float duty_ratio);
	void backward(float duty_ratio);
	void stop();
	void updateRotation(float factor);
	inline EMotorState getMotorState() {return motor_state; }
protected:
	uint8_t forward_channel;
	uint8_t backward_channel;
	EMotorState motor_state;	
	
	void ledcAnalogWrite(uint8_t channel, float duty_ratio);
	
};

#endif