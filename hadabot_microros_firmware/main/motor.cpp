#include "motor.h"
#include "Arduino.h"

#define LEDC_TIMER_13_BIT  13

//+++++++++++ Motor ++++=+++++++++++++++++++++++


Motor::Motor(uint8_t _forward_channel, uint8_t _backward_channel, uint8_t forward_pin_num, uint8_t backward_pin_num, double freq) :
forward_channel{_forward_channel}, backward_channel{_backward_channel}, motor_state{STOPED}
{
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
	motor_state = STOPED;
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



/*

enum EMotorState left_wheel_state;
enum EMotorState right_wheel_state;


int left_stoping_counter = 0;
int right_stoping_counter = 0;

static void mcpwm_gpio_initialize(void)
{
    printf("initializing mcpwm gpio...\n");
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, GPIO_PWM0A_OUT);
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0B, GPIO_PWM0B_OUT);
	
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM1A, GPIO_PWM1A_OUT);
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM1B, GPIO_PWM1B_OUT);
	
}


static void mcpwm_brushed_motor_control_init(void *arg)
{
    //1. mcpwm gpio initialization
    mcpwm_gpio_initialize();
	
	left_wheel_state = STOPED;
	right_wheel_state = STOPED;

    //2. initial mcpwm configuration
    printf("Configuring Initial Parameters of mcpwm...\n");
    mcpwm_config_t pwm_config;
    pwm_config.frequency = 1000;    //frequency = 500Hz,
    pwm_config.cmpr_a = 0;    //duty cycle of PWMxA = 0
    pwm_config.cmpr_b = 0;    //duty cycle of PWMxb = 0
    pwm_config.counter_mode = MCPWM_UP_COUNTER;
    pwm_config.duty_mode = MCPWM_DUTY_MODE_0;
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);    //Configure PWM0A & PWM0B with above settings
	mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_1, &pwm_config); 
	
}


static void brushed_motor_forward(mcpwm_unit_t mcpwm_num, mcpwm_timer_t timer_num , float duty_cycle)
{
    mcpwm_set_signal_low(mcpwm_num, timer_num, MCPWM_OPR_B);
    mcpwm_set_duty(mcpwm_num, timer_num, MCPWM_OPR_A, duty_cycle);
    mcpwm_set_duty_type(mcpwm_num, timer_num, MCPWM_OPR_A, MCPWM_DUTY_MODE_0); //call this each time, if operator was previously in low/high state
}


static void brushed_motor_backward(mcpwm_unit_t mcpwm_num, mcpwm_timer_t timer_num , float duty_cycle)
{
    mcpwm_set_signal_low(mcpwm_num, timer_num, MCPWM_OPR_A);
    mcpwm_set_duty(mcpwm_num, timer_num, MCPWM_OPR_B, duty_cycle);
    mcpwm_set_duty_type(mcpwm_num, timer_num, MCPWM_OPR_B, MCPWM_DUTY_MODE_0);  //call this each time, if operator was previously in low/high state
}


static void brushed_motor_stop(mcpwm_unit_t mcpwm_num, mcpwm_timer_t timer_num)
{
    mcpwm_set_signal_low(mcpwm_num, timer_num, MCPWM_OPR_A);
    mcpwm_set_signal_low(mcpwm_num, timer_num, MCPWM_OPR_B);
}



// Нужно сделать применимой для обоих моторов
void updateMotorRotation(mcpwm_unit_t mcpwm_num, mcpwm_timer_t timer_num, float duty) {
		if (duty > 0) {
			brushed_motor_forward(mcpwm_num, timer_num, duty);
		} else 
			if (duty < 0) {
				brushed_motor_backward(mcpwm_num, timer_num, (-1.0)*duty);
			} else {
				brushed_motor_stop(mcpwm_num, timer_num);
			}	
}


void updateMotorState(mcpwm_unit_t mcpwm_num, 
						mcpwm_timer_t timer_num, 
						enum EMotorState* cur_wheel_state, 
						int *stoping_counter,
						float factor) {
		enum EMotorState new_wheel_state;
		
		float duty = factor * 100.0;
		if (duty > 100.0) duty = 100.0;
		if (duty < -100) duty = -100.0;
		
		if (duty > 0) { new_wheel_state = FORWARD; *stoping_counter = 0; }
		else 
			if (duty < 0) { new_wheel_state = BACKWARD;  *stoping_counter = 0;}
			else {
				if (*cur_wheel_state == FORWARD) {
					new_wheel_state = STOPING_FORWARD;
					*stoping_counter = 15;
				}
				if (*cur_wheel_state == BACKWARD) {
					new_wheel_state = STOPING_BACKWARD;
					*stoping_counter = 15;		
				}					
			}
		
		updateMotorRotation(mcpwm_num, timer_num, duty);
		*cur_wheel_state = new_wheel_state;
}

void updateStopingSubstate(enum EMotorState* cur_wheel_state, int *stoping_counter) {
	(*stoping_counter)--;
	if ((*stoping_counter) == 0) 
		(*cur_wheel_state) = STOPED;
}
*/

