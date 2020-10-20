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


#include "esp_attr.h"

#include "driver/mcpwm.h"
#include "soc/mcpwm_periph.h"
#include "driver/gpio.h"



#define GPIO_PWM0A_OUT 25  //Set GPIO 15 as PWM0A
#define GPIO_PWM0B_OUT 26   //Set GPIO 16 as PWM0B

#define GPIO_PWM1A_OUT 32  
#define GPIO_PWM1B_OUT 33  


#define LOG_INFO_MSG_SIZE 200



enum EWheelRotationState {STOPED = 0, FORWARD = 1, BACKWARD = -1, STOPING_FORWARD = 2, STOPING_BACKWARD = -2};



void updateMotorState(mcpwm_unit_t mcpwm_num, 
						mcpwm_timer_t timer_num, 
						enum EWheelRotationState* cur_wheel_state, 
						int *stoping_counter,
						float factor);

void updateStopingSubstate(enum EWheelRotationState* cur_wheel_state, int *stoping_counter);



