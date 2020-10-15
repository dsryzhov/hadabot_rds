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
#include "driver/timer.h"

#define TIMER_DIVIDER         16  //  Hardware timer clock divider
#define TIMER_SCALE           (TIMER_BASE_CLK / TIMER_DIVIDER)  // convert counter value to seconds


#define GPIO_PWM0A_OUT 25  //Set GPIO 15 as PWM0A
#define GPIO_PWM0B_OUT 26   //Set GPIO 16 as PWM0B

#define GPIO_PWM1A_OUT 32  
#define GPIO_PWM1B_OUT 33  


#define LOG_INFO_MSG_SIZE 200



enum EWheelRotationState {STOPED = 0, FORWARD = 1, BACKWARD = -1, STOPING_FORWARD = 2, STOPING_BACKWARD = -2};

enum EWheelRotationState left_wheel_state;
enum EWheelRotationState right_wheel_state;



rcl_subscription_t subscriber_wheel_power_left;
rcl_subscription_t subscriber_wheel_power_right;

rcl_publisher_t log_info_publisher;
rcl_publisher_t wheel_radps_left_publisher;
rcl_publisher_t wheel_radps_right_publisher;

std_msgs__msg__Float32 wheel_power_left_msg;
std_msgs__msg__Float32 wheel_power_right_msg;
std_msgs__msg__Float32 wheel_radps_left_msg;
std_msgs__msg__Float32 wheel_radps_right_msg;
std_msgs__msg__String log_info_msg;


void log_info_timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
	UNUSED(last_call_time);

	if (timer != NULL) {
		// Fill the message timestamp
		struct timespec ts;
		clock_gettime(CLOCK_REALTIME, &ts);
		//sprintf(log_info_string, "Hadabot heartbeat %d_%d", ts.tv_sec, ts.tv_nsec);

		//sprintf(log_info_msg.data.data, "Hadabot heartbeat %d_%d", ts.tv_sec, ts.tv_nsec);
		sprintf(log_info_msg.data.data, "Hadabot heartbeat");
		log_info_msg.data.size = strlen(log_info_msg.data.data);
		
		
		// Reset the pong count and publish the ping message

		rcl_publish(&log_info_publisher, (const void*)&log_info_msg, NULL);
		printf("Sent Hadabot heartbeat\n");
	}
}



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

int left_stoping_counter = 0;
int right_stoping_counter = 0;

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

void left_motor_callback(const void * msgin)
{
	const std_msgs__msg__Float32 * msg = (const std_msgs__msg__Float32 *)msgin;
	printf("Received message /hadabot/wheel_power_left.");
	if (msg != NULL) {
		printf("Data: %f\n", msg->data);
		
		updateMotorState(MCPWM_UNIT_0, MCPWM_TIMER_0, &left_wheel_state, &left_stoping_counter, msg->data);
	}
}

void right_motor_callback(const void * msgin)
{
	const std_msgs__msg__Float32 * msg = (const std_msgs__msg__Float32 *)msgin;
	printf("Received message /hadabot/wheel_power_right.");
	if (msg != NULL) {
		printf("Data: %f\n", msg->data);
		
		updateMotorState(MCPWM_UNIT_0, MCPWM_TIMER_1, &right_wheel_state, &right_stoping_counter, msg->data);
	}
}


// ++++++++++++++++++++++++++ Измерение радиальной скорости

#define LW_SENSOR_GPIO     4
#define RW_SENSOR_GPIO     2
#define GPIO_INPUT_PIN_SEL  ((1ULL<<LW_SENSOR_GPIO) | (1ULL<<RW_SENSOR_GPIO))
#define ESP_INTR_FLAG_DEFAULT 0

struct timespec left_wheel_sensor_change_last_time;
struct timespec right_wheel_sensor_change_last_time;
struct timespec left_wheel_sensor_change_time;
struct timespec right_wheel_sensor_change_time;

bool left_wheel_sensor_state;
bool right_wheel_sensor_state;
static xQueueHandle left_wheel_gpio_evt_queue = NULL;
static xQueueHandle right_wheel_gpio_evt_queue = NULL;


bool left_wheel_zero_radps = true;
bool right_wheel_zero_radps = true;

double left_wheel_last_time_sec = 0;
double right_wheel_last_time_sec = 0;

// Стоит уйти от глобальных переменных, содержащих время
// Вместо этого считать и сохранять delta time с последнего изменения в очереди
static void IRAM_ATTR left_wheel_gpio_isr_handler(void* arg)
{
    uint32_t gpio_num = (uint32_t) arg;
	bool new_sensor_state = gpio_get_level(gpio_num);
//	if (new_sensor_state != left_wheel_sensor_state) {
		
		timer_spinlock_take(TIMER_GROUP_0);
		int timer_idx = TIMER_0;
		uint64_t timer_counter_value = timer_group_get_counter_value_in_isr(TIMER_GROUP_0, timer_idx);


		double cur_time_sec = (double) timer_counter_value / TIMER_SCALE;
		
		double delta_time_sec = cur_time_sec - left_wheel_last_time_sec;
		
		if (delta_time_sec > 0.005 && left_wheel_sensor_state != new_sensor_state) {
	
			left_wheel_last_time_sec = cur_time_sec;
			left_wheel_sensor_state = new_sensor_state;				
			xQueueSendFromISR(left_wheel_gpio_evt_queue, &delta_time_sec, NULL);		
		} else {
			left_wheel_last_time_sec = cur_time_sec;
			left_wheel_sensor_state = new_sensor_state;		
		}
		
		timer_spinlock_give(TIMER_GROUP_0);			
		
		
//	}
}



static void IRAM_ATTR right_wheel_gpio_isr_handler(void* arg)
{
    uint32_t gpio_num = (uint32_t) arg;
	bool new_sensor_state = gpio_get_level(gpio_num);
	double delta_time_sec = 0;
//	if (new_sensor_state != right_wheel_sensor_state) {
		timer_spinlock_take(TIMER_GROUP_1);
		int timer_idx = TIMER_0;
		uint64_t timer_counter_value = timer_group_get_counter_value_in_isr(TIMER_GROUP_1, timer_idx);

				

		double cur_time_sec = (double) timer_counter_value / TIMER_SCALE;
		
		delta_time_sec = cur_time_sec - right_wheel_last_time_sec;

		if (delta_time_sec > 0.005 && right_wheel_sensor_state != new_sensor_state) {
		
			right_wheel_last_time_sec = cur_time_sec;
			right_wheel_sensor_state = new_sensor_state;	
		
			xQueueSendFromISR(right_wheel_gpio_evt_queue, &delta_time_sec, NULL);		
		} else {
			right_wheel_last_time_sec = cur_time_sec;
			right_wheel_sensor_state = new_sensor_state;	
		}
		timer_spinlock_give(TIMER_GROUP_1);			

//	}
}



static void left_wheel_radsp_calc_task(void* arg)
{
    double delta_time_sec;
    for(;;) {
        if(xQueueReceive(left_wheel_gpio_evt_queue, &delta_time_sec, portMAX_DELAY)) {
			
			if (delta_time_sec > 0.0) {
				
				float koef = 2*3.141596 / 20.0 / delta_time_sec;
				if (left_wheel_sensor_state == true) koef*=13.0/21.0;  // текущий уровень сигнала сенсора - высокий, т.е. отверстие. Значит прошедший соответствует перегородке, которая шире отверстия.
				else koef *= 8.0/21.0;


				int sign = 1;
				if (left_wheel_state < 0) sign = -1;
			
				if (left_wheel_state != STOPED)
					koef = koef * sign;	
				else koef = 0;
				
				//if (koef < 20)
					wheel_radps_left_msg.data = koef;
				
				//printf("delta_time : %f  radsp : %f\n", (float)delta_time_sec, (float)koef);				

			} else wheel_radps_left_msg.data = 0;
			
		   
        }
    }
}

static void right_wheel_radsp_calc_task(void* arg)
{
    double delta_time_sec;
    for(;;) {
        if(xQueueReceive(right_wheel_gpio_evt_queue, &delta_time_sec, portMAX_DELAY)) {
			
			if (delta_time_sec > 0.0) {
				float koef = 2*3.141596 / 20.0 / delta_time_sec;
				if (right_wheel_sensor_state == true) koef*=123.0/210.0;  // текущий уровень сигнала сенсора - высокий, т.е. отверстие. Значит прошедший соответствует перегородке, которая шире отверстия.
				else koef *= 87.0/210.0;

				//printf("delta_time : %f  radsp : %f\n", (float)delta_time_sec, (float)koef);
				
				int sign = 1;
				if (right_wheel_state < 0) sign = -1;
				
				if (right_wheel_state != STOPED)
				   koef = koef * sign;	
			

				//if (koef < 20)
					wheel_radps_right_msg.data = koef;

			} else 
				wheel_radps_right_msg.data = 0;
	   
        }
    }
}

float wheel_radps_left_prev = 0;
float wheel_radps_right_prev = 0;

// Не правильно отслеживать последнее совпадение здесь

void wheel_radsp_timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
	UNUSED(last_call_time);

	
	if (timer != NULL) {
		if (left_wheel_state != STOPED) {
			
			if (left_wheel_state == STOPING_FORWARD || left_wheel_state == STOPING_BACKWARD) {
				updateStopingSubstate(&left_wheel_state, &left_stoping_counter);
			
				//if (wheel_radps_left_msg.data == wheel_radps_left_prev && abs(wheel_radps_left_msg.data) < 6) {
				if (abs(wheel_radps_left_msg.data) < 6) {
					wheel_radps_left_msg.data = 0;
					left_stoping_counter = 0;
					left_wheel_state = STOPED;
				}
			}
			
			rcl_publish(&wheel_radps_left_publisher, (const void*)&wheel_radps_left_msg, NULL);		
			wheel_radps_left_prev = wheel_radps_left_msg.data;

		}			
		if (right_wheel_state != STOPED) {
			if (right_wheel_state == STOPING_FORWARD || right_wheel_state == STOPING_BACKWARD) {
				updateStopingSubstate(&right_wheel_state, &right_stoping_counter);
				
				//if (wheel_radps_right_prev == wheel_radps_right_msg.data && abs(wheel_radps_right_msg.data) < 6) {
				if (abs(wheel_radps_right_msg.data) < 6) {
					wheel_radps_right_msg.data = 0;
					right_stoping_counter = 0;
					right_wheel_state = STOPED;
				}

			}
				
			rcl_publish(&wheel_radps_right_publisher, (const void*)&wheel_radps_right_msg, NULL);
			wheel_radps_right_prev = wheel_radps_right_msg.data;
		}			
		

/*		
		// Команда по остановке мотора еще не значит, что машинка остановилась
		// Но пока исходим из этого предположения
		// Потом нужно будет отслеживать остановку по IMU или еще как то.
		
	

		if (left_wheel_state == STOPED) wheel_radps_left_msg.data = 0.0;		
		if (right_wheel_state == STOPED) wheel_radps_right_msg.data = 0.0;


		bool flLP = false;
		if (wheel_radps_left_msg.data != 0.0) {flLP = true; left_wheel_zero_radps = false;}
		else
			if (!left_wheel_zero_radps) {flLP = true; left_wheel_zero_radps = true;}

		if (flLP)  {
			rcl_publish(&wheel_radps_left_publisher, (const void*)&wheel_radps_left_msg, NULL);		
		}

		bool flRP = false;
		if (wheel_radps_right_msg.data != 0.0) {flRP = true; right_wheel_zero_radps = false;}
		else
			if (!right_wheel_zero_radps) {flRP = true; right_wheel_zero_radps = true;}

		if (flRP)  {
			rcl_publish(&wheel_radps_right_publisher, (const void*)&wheel_radps_right_msg, NULL);
		}
*/	
	}

}

void wheel_sensors_init() {
    gpio_config_t io_conf;
    //disable interrupt
    io_conf.intr_type = GPIO_PIN_INTR_DISABLE;

    //interrupt of rising edge
    io_conf.intr_type = GPIO_PIN_INTR_ANYEDGE;//GPIO_PIN_INTR_ANYEDGE;
    //bit mask of the pins, use GPIO4/5 here
    io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL;
    //set as input mode    
    io_conf.mode = GPIO_MODE_INPUT;
    //enable pull-up mode
//    io_conf.pull_up_en = 0;
	io_conf.pull_down_en = 0;
    gpio_config(&io_conf);

    //change gpio intrrupt type for one pin
//    gpio_set_intr_type(LW_SENSOR_GPIO, GPIO_INTR_ANYEDGE);

    //create a queue to handle gpio event from isr
    left_wheel_gpio_evt_queue = xQueueCreate(10, sizeof(double));
	right_wheel_gpio_evt_queue = xQueueCreate(10, sizeof(double));
    //start gpio task
	
	
    xTaskCreate(left_wheel_radsp_calc_task, "left_wheel_radsp_calc_task", 6144, NULL, 10, NULL);
    xTaskCreate(right_wheel_radsp_calc_task, "right_wheel_radsp_calc_task", 6144, NULL, 10, NULL);	


	left_wheel_sensor_state = gpio_get_level(LW_SENSOR_GPIO);
	right_wheel_sensor_state = gpio_get_level(RW_SENSOR_GPIO);
	
	clock_gettime(CLOCK_REALTIME, &left_wheel_sensor_change_time);
	right_wheel_sensor_change_time = left_wheel_sensor_change_time;

    //install gpio isr service
    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
    //hook isr handler for specific gpio pin
    gpio_isr_handler_add(LW_SENSOR_GPIO, left_wheel_gpio_isr_handler, (void*) LW_SENSOR_GPIO);
    //hook isr handler for specific gpio pin
    gpio_isr_handler_add(RW_SENSOR_GPIO, right_wheel_gpio_isr_handler, (void*) RW_SENSOR_GPIO);	
}

/*
 * Initialize selected timer of the timer group 0
 *
 * timer_idx - the timer number to initialize
 * auto_reload - should the timer auto reload on alarm?
 */


static void tg0_timer_init(timer_group_t timer_group, int timer_idx)
{

    timer_config_t config = {
        .divider = TIMER_DIVIDER,
        .counter_dir = TIMER_COUNT_UP,
        .counter_en = TIMER_PAUSE,
        .alarm_en = TIMER_ALARM_DIS,
        .auto_reload = TIMER_AUTORELOAD_DIS,
    }; // default clock source is APB
    timer_init(timer_group, timer_idx, &config);

    timer_set_counter_value(timer_group, timer_idx, 0);
    timer_start(timer_group, timer_idx);
}



void appMain(void * arg)
{
	mcpwm_brushed_motor_control_init(arg);
	
	tg0_timer_init(TIMER_GROUP_0, TIMER_0);
	tg0_timer_init(TIMER_GROUP_1, TIMER_0);
	wheel_sensors_init();
	
  	rcl_allocator_t allocator = rcl_get_default_allocator();
	rclc_support_t support;

	// create init_options
	RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

	// create node
	rcl_node_t node = rcl_get_zero_initialized_node();
	RCCHECK(rclc_node_init_default(&node, "hadabot_esp32", "", &support));

	RCCHECK(rclc_publisher_init_default(&log_info_publisher, &node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), "hadabot/log/info"));

	rcl_timer_t log_info_timer = rcl_get_zero_initialized_timer();
	RCCHECK(rclc_timer_init_default(&log_info_timer, &support, RCL_MS_TO_NS(5000), log_info_timer_callback));
	
	rcl_timer_t wheel_radsp_timer = rcl_get_zero_initialized_timer();
	RCCHECK(rclc_timer_init_default(&wheel_radsp_timer, &support, RCL_MS_TO_NS(15), wheel_radsp_timer_callback));
	
	
	RCCHECK(rclc_publisher_init_default(&wheel_radps_left_publisher, &node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "hadabot/wheel_radps_left"));

	RCCHECK(rclc_publisher_init_default(&wheel_radps_right_publisher, &node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "hadabot/wheel_radps_right"));
		
		
	// create subscriber for wheel power left message
	
	RCCHECK(rclc_subscription_init_default(
	//RCCHECK(rclc_subscription_init_best_effort(	
		&subscriber_wheel_power_left,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
		"/hadabot/wheel_power_left"));


// create subscriber for wheel power right message

	RCCHECK(rclc_subscription_init_default(
//	RCCHECK(rclc_subscription_init_best_effort(
		&subscriber_wheel_power_right,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
		"/hadabot/wheel_power_right"));
		
	
	// create executor
	rclc_executor_t executor = rclc_executor_get_zero_initialized_executor();
	RCCHECK(rclc_executor_init(&executor, &support.context, 4, &allocator));

	unsigned int rcl_wait_timeout = 100;   // in ms
	RCCHECK(rclc_executor_set_timeout(&executor, RCL_MS_TO_NS(rcl_wait_timeout)));
	RCCHECK(rclc_executor_add_timer(&executor, &wheel_radsp_timer));		
	RCCHECK(rclc_executor_add_timer(&executor, &log_info_timer));
	RCCHECK(rclc_executor_add_subscription(&executor, &subscriber_wheel_power_left, &wheel_power_left_msg, &left_motor_callback, ON_NEW_DATA));
	RCCHECK(rclc_executor_add_subscription(&executor, &subscriber_wheel_power_right, &wheel_power_right_msg, &right_motor_callback, ON_NEW_DATA));

	log_info_msg.data.data = (char * ) malloc(LOG_INFO_MSG_SIZE * sizeof(char));
	log_info_msg.data.size = 0;
	log_info_msg.data.capacity = LOG_INFO_MSG_SIZE;

	
	rclc_executor_spin(&executor);

	RCCHECK(rcl_publisher_fini(&log_info_publisher, &node));
	RCCHECK(rcl_publisher_fini(&wheel_radps_left_publisher, &node));
	RCCHECK(rcl_publisher_fini(&wheel_radps_right_publisher, &node));
	RCCHECK(rcl_subscription_fini(&subscriber_wheel_power_left, &node));
	RCCHECK(rcl_subscription_fini(&subscriber_wheel_power_right, &node));
	RCCHECK(rcl_node_fini(&node));
	vTaskDelete(NULL);
}
