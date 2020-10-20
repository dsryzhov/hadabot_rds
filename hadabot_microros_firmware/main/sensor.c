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


void wheel_sensors_init() {
	
	tg0_timer_init(TIMER_GROUP_0, TIMER_0);
	tg0_timer_init(TIMER_GROUP_1, TIMER_0);
	
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





