#include "motor.h"

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/header.h>
#include <std_msgs/msg/string.h>
#include <std_msgs/msg/float32.h>
#include <geometry_msgs/msg/twist.h>
#include <stdio.h>
#include <unistd.h>
#include <time.h>

#ifdef ESP_PLATFORM
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#endif

#define STRING_BUFFER_LEN 50

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc);vTaskDelete(NULL);}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}

#include "esp_attr.h"

#include "driver/mcpwm.h"
#include "soc/mcpwm_periph.h"
//#include "driver/pcnt.h"
//#include "driver/periph_ctrl.h"
#include "driver/gpio.h"

enum EWheelRotationState left_wheel_state;
enum EWheelRotationState right_wheel_state;


int left_stoping_counter = 0;
int right_stoping_counter = 0;

//+++++++++++ Motor ++++=+++++++++++++++++++++++


static void mcpwm_gpio_initialize(void)
{
    printf("initializing mcpwm gpio...\n");
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, GPIO_PWM0A_OUT);
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0B, GPIO_PWM0B_OUT);
	
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM1A, GPIO_PWM1A_OUT);
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM1B, GPIO_PWM1B_OUT);
	
}


/**
 * @brief Configure MCPWM module for brushed dc motor
 */
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


/**
 * @brief motor moves in forward direction, with duty cycle = duty %
 */
static void brushed_motor_forward(mcpwm_unit_t mcpwm_num, mcpwm_timer_t timer_num , float duty_cycle)
{
    mcpwm_set_signal_low(mcpwm_num, timer_num, MCPWM_OPR_B);
    mcpwm_set_duty(mcpwm_num, timer_num, MCPWM_OPR_A, duty_cycle);
    mcpwm_set_duty_type(mcpwm_num, timer_num, MCPWM_OPR_A, MCPWM_DUTY_MODE_0); //call this each time, if operator was previously in low/high state
}

/**
 * @brief motor moves in backward direction, with duty cycle = duty %
 */
static void brushed_motor_backward(mcpwm_unit_t mcpwm_num, mcpwm_timer_t timer_num , float duty_cycle)
{
    mcpwm_set_signal_low(mcpwm_num, timer_num, MCPWM_OPR_A);
    mcpwm_set_duty(mcpwm_num, timer_num, MCPWM_OPR_B, duty_cycle);
    mcpwm_set_duty_type(mcpwm_num, timer_num, MCPWM_OPR_B, MCPWM_DUTY_MODE_0);  //call this each time, if operator was previously in low/high state
}

/**
 * @brief motor stop
 */
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
						enum EWheelRotationState* cur_wheel_state, 
						int *stoping_counter,
						float factor) {
		enum EWheelRotationState new_wheel_state;
		
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

void updateStopingSubstate(enum EWheelRotationState* cur_wheel_state, int *stoping_counter) {
	(*stoping_counter)--;
	if ((*stoping_counter) == 0) 
		(*cur_wheel_state) = STOPED;
}


