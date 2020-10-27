#include "sdkconfig.h"

#include "uxr/client/config.h"
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


#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc);vTaskDelete(NULL);}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}

//#include "esp_attr.h"
//#include "driver/gpio.h"
//#include "driver/timer.h"


#include "hadabot_hw.h"
//#include "rotsensor.h"
//#include "motor.h"

HadabotHW* pHadabotHW;


#define LOG_INFO_MSG_SIZE 200
rcl_publisher_t log_info_publisher;
std_msgs__msg__String log_info_msg;

rcl_subscription_t subscriber_wheel_power_left;
rcl_subscription_t subscriber_wheel_power_right;
std_msgs__msg__Float32 wheel_power_left_msg;
std_msgs__msg__Float32 wheel_power_right_msg;

rcl_publisher_t wheel_radps_left_publisher;
rcl_publisher_t wheel_radps_right_publisher;
std_msgs__msg__Float32 wheel_radps_left_msg;
std_msgs__msg__Float32 wheel_radps_right_msg;

rcl_publisher_t distance_publisher;
std_msgs__msg__Float32 distance_msg;


void log_info_timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
	UNUSED(last_call_time);

	if (timer != NULL) {
		// Fill the message timestamp
		struct timespec ts;
		clock_gettime(CLOCK_REALTIME, &ts);
		sprintf(log_info_msg.data.data, "Hadabot heartbeat %ld", ts.tv_sec);
		//sprintf(log_info_msg.data.data, "Hadabot heartbeat");
		log_info_msg.data.size = strlen(log_info_msg.data.data);

		rcl_publish(&log_info_publisher, (const void*)&log_info_msg, NULL);
		printf(log_info_msg.data.data);
		printf("\n");
	}
}

void wheel_power_left_callback(const void * msgin)
{
	const std_msgs__msg__Float32 * msg = (const std_msgs__msg__Float32 *)msgin;
	printf("Received message /hadabot/wheel_power_left.");
	if (msg != NULL) {
		printf("Data: %f\n", msg->data);
		pHadabotHW->getLeftMotor()->updateRotation(msg->data);
	}
}

void wheel_power_right_callback(const void * msgin)
{
	const std_msgs__msg__Float32 * msg = (const std_msgs__msg__Float32 *)msgin;
	printf("Received message /hadabot/wheel_power_right.");
	if (msg != NULL) {
		printf("Data: %f\n", msg->data);
		pHadabotHW->getRightMotor()->updateRotation(msg->data);
	}
}

void wheel_radsp_timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
	UNUSED(last_call_time);
	if (timer != NULL) {
		wheel_radps_left_msg.data = pHadabotHW->getLeftWheelRotationSensor()->getAngularVelocity();
		wheel_radps_right_msg.data = pHadabotHW->getRightWheelRotationSensor()->getAngularVelocity();
		if (wheel_radps_left_msg.data != 0 || wheel_radps_right_msg.data != 0) {
			rcl_publish(&wheel_radps_left_publisher, (const void*)&wheel_radps_left_msg, NULL);	
			rcl_publish(&wheel_radps_right_publisher, (const void*)&wheel_radps_right_msg, NULL);			
		}
	}		
}

void distance_timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
	UNUSED(last_call_time);
	
	if (timer != NULL) {
		
		distance_msg.data = pHadabotHW->getHCSR04()->getDistanceInMillimeters();
		
		if (distance_msg.data != -1 ) {

			rcl_publish(&distance_publisher, (const void*)&distance_msg, NULL);	
		}
	}		
}


extern "C"  void appMain(void * arg)
{
	log_info_msg.data.data = (char * ) malloc(LOG_INFO_MSG_SIZE * sizeof(char));
	log_info_msg.data.size = 0;
	log_info_msg.data.capacity = LOG_INFO_MSG_SIZE;
	
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
	RCCHECK(rclc_timer_init_default(&wheel_radsp_timer, &support, RCL_MS_TO_NS(CONFIG_HADABOT_WHEELS_RADPS_MSG_PUBLISH_PERIOD), wheel_radsp_timer_callback));

	rcl_timer_t distance_timer = rcl_get_zero_initialized_timer();
	RCCHECK(rclc_timer_init_default(&distance_timer, &support, RCL_MS_TO_NS(60), distance_timer_callback));
	
	
	RCCHECK(rclc_publisher_init_default(&wheel_radps_left_publisher, &node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "hadabot/wheel_radps_left"));

	RCCHECK(rclc_publisher_init_default(&wheel_radps_right_publisher, &node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "hadabot/wheel_radps_right"));

	RCCHECK(rclc_publisher_init_default(&distance_publisher, &node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "hadabot/distance_forward"));

		
	// create subscriber for wheel power left message
	
	RCCHECK(rclc_subscription_init_default(
		&subscriber_wheel_power_left,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
		"/hadabot/wheel_power_left"));


// create subscriber for wheel power right message

	RCCHECK(rclc_subscription_init_default(
		&subscriber_wheel_power_right,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
		"/hadabot/wheel_power_right"));
		

	// create executor
	rclc_executor_t executor = rclc_executor_get_zero_initialized_executor();
	RCCHECK(rclc_executor_init(&executor, &support.context, 5, &allocator));

	unsigned int rcl_wait_timeout = 100;   // timeout for waiting for new data from subscribed topics
	RCCHECK(rclc_executor_set_timeout(&executor, RCL_MS_TO_NS(rcl_wait_timeout)));

	RCCHECK(rclc_executor_add_subscription(&executor, &subscriber_wheel_power_left, &wheel_power_left_msg, &wheel_power_left_callback, ON_NEW_DATA));
	RCCHECK(rclc_executor_add_subscription(&executor, &subscriber_wheel_power_right, &wheel_power_right_msg, &wheel_power_right_callback, ON_NEW_DATA));
	RCCHECK(rclc_executor_add_timer(&executor, &wheel_radsp_timer));		
	RCCHECK(rclc_executor_add_timer(&executor, &distance_timer));
	RCCHECK(rclc_executor_add_timer(&executor, &log_info_timer));	

	
	rclc_executor_spin(&executor);

	RCCHECK(rcl_publisher_fini(&log_info_publisher, &node));
	
	RCCHECK(rcl_publisher_fini(&wheel_radps_left_publisher, &node));
	RCCHECK(rcl_publisher_fini(&wheel_radps_right_publisher, &node));
	RCCHECK(rcl_publisher_fini(&distance_publisher, &node));
		
	RCCHECK(rcl_subscription_fini(&subscriber_wheel_power_left, &node));
	RCCHECK(rcl_subscription_fini(&subscriber_wheel_power_right, &node));

	RCCHECK(rcl_node_fini(&node));
	vTaskDelete(NULL);
}

extern "C"  void microros_interface_init(HadabotHW* _pHadabotHW) {
	pHadabotHW = _pHadabotHW;
    // start microROS task
    xTaskCreate(appMain, "uros_task", CONFIG_MICRO_ROS_APP_STACK, NULL, 5, NULL);	
}