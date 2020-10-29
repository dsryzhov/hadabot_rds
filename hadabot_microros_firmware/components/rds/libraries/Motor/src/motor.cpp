#include "motor.h"
#include "Arduino.h"

#define LEDC_TIMER_13_BIT  13

//+++++++++++ Motor ++++=+++++++++++++++++++++++


Motor::Motor(const char* _name, uint8_t _forward_channel, uint8_t _backward_channel, uint8_t forward_pin_num, uint8_t backward_pin_num, double freq) :
name{_name}, forward_channel{_forward_channel}, backward_channel{_backward_channel}, motor_state{STOPED}
{

	gpio_set_direction((gpio_num_t)forward_pin_num, GPIO_MODE_OUTPUT);
	gpio_set_pull_mode((gpio_num_t)forward_pin_num, GPIO_PULLUP_ONLY);

	gpio_set_direction((gpio_num_t)backward_pin_num, GPIO_MODE_OUTPUT);
	gpio_set_pull_mode((gpio_num_t)backward_pin_num, GPIO_PULLUP_ONLY);	

	//double ledcSetup(uint8_t chan, double freq, uint8_t bit_num)	
	ledcSetup(forward_channel, freq, LEDC_TIMER_13_BIT);
	ledcAttachPin(forward_pin_num, forward_channel);
	
	ledcSetup(backward_channel, freq, LEDC_TIMER_13_BIT);
	ledcAttachPin(backward_pin_num, backward_channel);
  
}

Motor::~Motor() {
}

// Arduino like analogWrite
// value has to be between 0 and valueMax
void Motor::ledcAnalogWrite(uint8_t channel, float duty_ratio) {
  // calculate duty, 8191 from 2 ^ 13 - 1
  uint32_t duty = 8191*duty_ratio;

  // write duty to LEDC
  ledcWrite(channel, duty);
}
  
void Motor::forward(float duty_ratio) {
	ledcWrite(backward_channel, 0);			
	ledcAnalogWrite(forward_channel, duty_ratio);	
	motor_state = FORWARD;
}

void Motor::backward(float duty_ratio) {
	ledcWrite(forward_channel, 0);	
	ledcAnalogWrite(backward_channel, duty_ratio);		
	motor_state = BACKWARD;
}

void Motor::stop() {
	ledcWrite(backward_channel, 0);				
	ledcWrite(forward_channel, 0);	
	if (motor_state == FORWARD)
		motor_state = STOPING_FORWARD;
	else
		if (motor_state == BACKWARD)
			motor_state = STOPING_BACKWARD;
	
}

void Motor::evStoped() {
	if (motor_state == STOPING_FORWARD) {
		motor_state = STOPED;
		printf("%s stopped after moving forward. \n", name);		
	}
	else
		if (motor_state == STOPING_BACKWARD) {
			motor_state = STOPED;
			printf("%s stopped after moving backward. \n", name);		
		}	
}

void Motor::updateRotation(float factor) {
	if (factor > 0) {
		forward(factor);
	} else 
		if (factor < 0) {
			backward((-1.0)*factor);
		} else {
			stop();
		}	
}
