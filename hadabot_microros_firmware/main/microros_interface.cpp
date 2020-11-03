#include "sdkconfig.h"

#include "uxr/client/config.h"
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/header.h>
#include <std_msgs/msg/string.h>
#include <std_msgs/msg/float32.h>
#include <sensor_msgs/msg/temperature.h>
#include <geometry_msgs/msg/twist.h>
#include <geometry_msgs/msg/pose2_d.h>
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

#define FRAME_ID_SIZE 10
#define LOG_INFO_MSG_SIZE 200

HadabotHW* pHadabotHW;

rcl_clock_t ros_clock;


rcl_publisher_t log_info_publisher;
std_msgs__msg__String log_info_msg;

rcl_subscription_t subscriber_wheel_power_left;
rcl_subscription_t subscriber_wheel_power_right;
std_msgs__msg__Float32 wheel_power_left_msg;
std_msgs__msg__Float32 wheel_power_right_msg;

rcl_publisher_t wheel_radps_left_publisher;
rcl_publisher_t wheel_radps_right_publisher;

sensor_msgs__msg__Temperature wheel_radps_left_msg;
sensor_msgs__msg__Temperature wheel_radps_right_msg;

//std_msgs__msg__Float32 wheel_radps_left_msg;
//std_msgs__msg__Float32 wheel_radps_right_msg;

rcl_publisher_t position_publisher;
geometry_msgs__msg__Pose2D position_msg;

rcl_publisher_t distance_publisher;
//std_msgs__msg__Float32 distance_msg;
sensor_msgs__msg__Temperature distance_msg;

xQueueHandle distance_evt_queue = NULL;
xQueueHandle wheel_radsp_left_evt_queue = NULL;
xQueueHandle wheel_radsp_right_evt_queue = NULL;

void log_info_timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
	UNUSED(last_call_time);

	if (timer != NULL) {
		// Fill the message timestamp
		/*
		struct timespec ts;
		clock_gettime(CLOCK_REALTIME, &ts);
		sprintf(log_info_msg.data.data, "Hadabot heartbeat %ld", ts.tv_sec);
		*/

		rcl_time_point_value_t base_time_point_value;
		rcl_clock_get_now(&ros_clock, &base_time_point_value);
		double time_sec =  base_time_point_value / 1000000000.0;
		sprintf(log_info_msg.data.data, "Hadabot heartbeat %lf", time_sec);


	 //fprintf(stdout, "Cur time ns %llu, msg time ns %llu, difference %lld / %ld\n",
    //    curtime, msgtime, duration_ns, duration_ms);		

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
		printf(" with Data: %f\n", msg->data);
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

bool flRadspsLeftZeroPublished = false;
bool flRadspsRightZeroPublished = false;

void sensor_data_publish_timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
	UNUSED(last_call_time);
	PosEstimator *pPosEstimator = pHadabotHW->getPosEstimator();
	Position pos;
	pPosEstimator->getPosition(pos);
	//printf("x: %f   y:   %f    theta   %f\n", pos.x, pos.y, pos.theta);

	position_msg.x = pos.x;
	position_msg.y = pos.y;
	position_msg.theta = pos.theta;

	rcl_publish(&position_publisher, (const void*)&position_msg, NULL);	


	//MPU6050* pMpu = pHadabotHW->getMPU6050();
	//pMpu->Execute();

//	pPosEstimator->updateMpuAngles();
//	float  theta = pPosEstimator->getMpuYaw();

//	printf("theta: %f\n ", theta);


/*
	if (timer != NULL) {
		int sec = 0;
		unsigned int nanosec = 0;
		float val = 0;
		pHadabotHW->getLeftWheelRotationSensor()->getAngularVelocity(&val, &sec, &nanosec);
		//int sensor_level = pHadabotHW->getLeftWheelRotationSensor()->getSensorLevel();
		//printf("sensor_level : %d\n", sensor_level);			

		wheel_radps_left_msg.temperature = val;
		wheel_radps_left_msg.variance = 0;
		wheel_radps_left_msg.header.stamp.sec = sec;
		wheel_radps_left_msg.header.stamp.nanosec = nanosec;
		

		pHadabotHW->getRightWheelRotationSensor()->getAngularVelocity(&val, &sec, &nanosec);
		wheel_radps_right_msg.temperature = val;
		wheel_radps_right_msg.variance = 0;
		wheel_radps_right_msg.header.stamp.sec = sec;
		wheel_radps_right_msg.header.stamp.nanosec = nanosec;

		distance_msg.temperature = pHadabotHW->getHCSR04()->getDistanceInMillimeters();
		distance_msg.variance = 0;
		distance_msg.header.stamp.sec = distance_msg.header.stamp.nanosec = 0;
		
		

		if (wheel_radps_left_msg.temperature != 0 || (wheel_radps_left_msg.temperature == 0 && !flRadspsLeftZeroPublished)) {
			rcl_publish(&wheel_radps_left_publisher, (const void*)&wheel_radps_left_msg, NULL);	
			if (wheel_radps_left_msg.temperature  == 0) flRadspsLeftZeroPublished = true;
			else flRadspsLeftZeroPublished = false;
		}
		
		
		if (wheel_radps_right_msg.temperature != 0 || (wheel_radps_right_msg.temperature == 0 && !flRadspsRightZeroPublished)) {		
			//printf("val : %f  sec: %d  nanosec %d\n", val, sec, nanosec);			
			rcl_publish(&wheel_radps_right_publisher, (const void*)&wheel_radps_right_msg, NULL);			
			if (wheel_radps_right_msg.temperature  == 0) flRadspsRightZeroPublished = true;
			else flRadspsRightZeroPublished = false;

		}
			
		rcl_publish(&distance_publisher, (const void*)&distance_msg, NULL);			
	}	
*/	
}


/*
void sensor_data_publish_timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
	UNUSED(last_call_time);
	
	if (timer != NULL) {
		
		wheel_radps_left_msg.data = pHadabotHW->getLeftWheelRotationSensor()->getAngularVelocity();
		wheel_radps_right_msg.data = pHadabotHW->getRightWheelRotationSensor()->getAngularVelocity();
		distance_msg.data = pHadabotHW->getHCSR04()->getDistanceInMillimeters();
		
		if (wheel_radps_left_msg.data != 0 || (wheel_radps_left_msg.data == 0 && !flRadspsLeftZeroPublished)) {
			rcl_publish(&wheel_radps_left_publisher, (const void*)&wheel_radps_left_msg, NULL);	
			if (wheel_radps_left_msg.data  == 0) flRadspsLeftZeroPublished = true;
			else flRadspsLeftZeroPublished = false;
		}
		
		if (wheel_radps_right_msg.data != 0 || (wheel_radps_right_msg.data == 0 && !flRadspsRightZeroPublished)) {		
			rcl_publish(&wheel_radps_right_publisher, (const void*)&wheel_radps_right_msg, NULL);			
			if (wheel_radps_right_msg.data  == 0) flRadspsRightZeroPublished = true;
			else flRadspsRightZeroPublished = false;

		}
			
		rcl_publish(&distance_publisher, (const void*)&distance_msg, NULL);			
	}		
}
*/

/*

void wheel_radsp_left_callback(float angular_velocity) {
	if (wheel_radsp_left_evt_queue != NULL)
		if (xQueueSend(wheel_radsp_left_evt_queue, &angular_velocity, ( TickType_t ) 0) != pdTRUE) 
			printf("Error: wheel_radsp_left_evt_queue is full\n");
}

void wheel_radsp_right_callback(float angular_velocity) {
	if (wheel_radsp_right_evt_queue != NULL)	
		if (xQueueSend(wheel_radsp_right_evt_queue, &angular_velocity, ( TickType_t ) 0) != pdTRUE) 
			printf("Error: wheel_radsp_right_evt_queue is full\n");
}

void distance_measured_callback(int distance) {
	if (wheel_radsp_left_evt_queue != NULL)		
		if (xQueueSend(distance_evt_queue, &distance, ( TickType_t ) 0) != pdTRUE) 
			printf("Error: distance_evt_queue is full\n");
}

void creare_publish_queues() {
	distance_evt_queue = xQueueCreate( 100, sizeof( int ) );
	wheel_radsp_left_evt_queue = xQueueCreate( 100, sizeof( float ) );
	wheel_radsp_right_evt_queue = xQueueCreate( 100, sizeof( float ) );

	pHadabotHW->getHCSR04()->setDistanceMeasuredCallback(&distance_measured_callback);
	pHadabotHW->getLeftWheelRotationSensor()->setDataUpdatedCallback(&wheel_radsp_left_callback);
	pHadabotHW->getRightWheelRotationSensor()->setDataUpdatedCallback(&wheel_radsp_right_callback);
}

void publish_msgs() {
	
	float radsp_left, radsp_right;
	int distance_mm;
	bool flMsgPublished = true;
	int count = 0;

	while (flMsgPublished && count < 10) {
		flMsgPublished = false;

		if (distance_evt_queue != NULL && xQueueReceive(distance_evt_queue, &distance_mm, ( TickType_t ) 0) == pdTRUE ) {  
			distance_msg.data = distance_mm;
			rcl_publish(&distance_publisher, (const void*)&distance_msg, NULL);	
			//printf("Published distance %d\n", distance_mm);
			flMsgPublished = true;
		}

		if (wheel_radsp_left_evt_queue != NULL && xQueueReceive(wheel_radsp_left_evt_queue, &radsp_left, ( TickType_t ) 0) == pdTRUE ) {
			wheel_radps_left_msg.data = radsp_left;
			rcl_publish(&wheel_radps_left_publisher, (const void*)&wheel_radps_left_msg, NULL);	
			flMsgPublished = true;
		}

		if (wheel_radsp_right_evt_queue != NULL && xQueueReceive(wheel_radsp_right_evt_queue, &radsp_right, ( TickType_t ) 0) == pdTRUE ) {
			wheel_radps_right_msg.data = radsp_right;
			rcl_publish(&wheel_radps_right_publisher, (const void*)&wheel_radps_right_msg, NULL);			
			flMsgPublished = true;
		}
		count++;
	}
}


void delete_publish_queues() {
	pHadabotHW->getHCSR04()->setDistanceMeasuredCallback(NULL);
	pHadabotHW->getLeftWheelRotationSensor()->setDataUpdatedCallback(NULL);
	pHadabotHW->getRightWheelRotationSensor()->setDataUpdatedCallback(NULL);

	vQueueDelete(distance_evt_queue);
	vQueueDelete(wheel_radsp_left_evt_queue);
	vQueueDelete(wheel_radsp_right_evt_queue);
}

*/

void init_messages() {
	log_info_msg.data.data = (char * ) malloc(LOG_INFO_MSG_SIZE * sizeof(char));
	log_info_msg.data.size = 0;
	log_info_msg.data.capacity = LOG_INFO_MSG_SIZE;

/*
	wheel_radps_left_msg.header.frame_id.data = (char*)malloc(FRAME_ID_SIZE * sizeof(char));
	wheel_radps_left_msg.header.frame_id.size=0;
	wheel_radps_left_msg.header.frame_id.capacity=FRAME_ID_SIZE;
	sprintf(wheel_radps_left_msg.header.frame_id.data, "FramL");
	wheel_radps_left_msg.header.frame_id.size = strlen(wheel_radps_left_msg.header.frame_id.data);
	
	wheel_radps_right_msg.header.frame_id.data = (char*)malloc(FRAME_ID_SIZE * sizeof(char));
	wheel_radps_right_msg.header.frame_id.size=0;
	wheel_radps_right_msg.header.frame_id.capacity=FRAME_ID_SIZE;
	sprintf(wheel_radps_right_msg.header.frame_id.data, "FramR");
	wheel_radps_right_msg.header.frame_id.size = strlen(wheel_radps_right_msg.header.frame_id.data);	
*/

	distance_msg.header.frame_id.data = (char*)malloc(FRAME_ID_SIZE * sizeof(char));
	distance_msg.header.frame_id.size=0;
	distance_msg.header.frame_id.capacity=FRAME_ID_SIZE;	
	sprintf(distance_msg.header.frame_id.data, "FramD");	
	distance_msg.header.frame_id.size = strlen(distance_msg.header.frame_id.data);	

/*
	position_msg.header.frame_id.data = (char*)malloc(FRAME_ID_SIZE * sizeof(char));
	position_msg.header.frame_id.size=0;
	position_msg.header.frame_id.capacity=FRAME_ID_SIZE;	
	sprintf(position_msg.header.frame_id.data, "FramP");	
	position_msg.header.frame_id.size = strlen(position_msg.header.frame_id.data);	
*/
}


extern "C"  void appMain(void * arg)
{
	init_messages();

  	rcl_allocator_t allocator = rcl_get_default_allocator();
	rclc_support_t support;

	// create init_options
	RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

   
	// create node
	rcl_node_t node = rcl_get_zero_initialized_node();
	RCCHECK(rclc_node_init_default(&node, "hadabot_esp32", "", &support));

 	RCCHECK(rcl_ros_clock_init(&ros_clock, &allocator));


	RCCHECK(rclc_publisher_init_default(&log_info_publisher, &node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), "hadabot/log/info"));


	rcl_timer_t log_info_timer = rcl_get_zero_initialized_timer();
	RCCHECK(rclc_timer_init_default(&log_info_timer, &support, RCL_MS_TO_NS(5000), log_info_timer_callback));

	rcl_timer_t sensor_data_publish_timer = rcl_get_zero_initialized_timer();
	RCCHECK(rclc_timer_init_default(&sensor_data_publish_timer, &support, RCL_MS_TO_NS(500), sensor_data_publish_timer_callback));

	RCCHECK(rclc_publisher_init_default(&position_publisher, &node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Pose2D), "hadabot/pose2D"));

/*	
	RCCHECK(rclc_publisher_init_default(&wheel_radps_left_publisher, &node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Temperature), "hadabot/wheel_radps_left_timestamped"));


	RCCHECK(rclc_publisher_init_default(&wheel_radps_right_publisher, &node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Temperature), "hadabot/wheel_radps_right_timestamped"));
*/

	RCCHECK(rclc_publisher_init_default(&distance_publisher, &node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Temperature), "hadabot/distance_forward_timestamped"));

		
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
	RCCHECK(rclc_executor_init(&executor, &support.context, 4, &allocator));

	unsigned int rcl_wait_timeout = 100;   // timeout for waiting for new data from subscribed topics
	RCCHECK(rclc_executor_set_timeout(&executor, RCL_MS_TO_NS(rcl_wait_timeout)));

	RCCHECK(rclc_executor_add_timer(&executor, &sensor_data_publish_timer));
	RCCHECK(rclc_executor_add_subscription(&executor, &subscriber_wheel_power_left, &wheel_power_left_msg, &wheel_power_left_callback, ON_NEW_DATA));
	RCCHECK(rclc_executor_add_subscription(&executor, &subscriber_wheel_power_right, &wheel_power_right_msg, &wheel_power_right_callback, ON_NEW_DATA));
	RCCHECK(rclc_executor_add_timer(&executor, &log_info_timer));	

	rclc_executor_spin(&executor);	

/*
	creare_publish_queues();

 	const TickType_t xDelay = 1 / portTICK_PERIOD_MS;

 	long int spinStart, spinStop;


	while (true) {
		//spinStart = micros();
		rclc_executor_spin_some(&executor,100);
		publish_msgs();
		//spinStop = micros();
		//printf("Delta time : %ld\n", spinStop-spinStart);		
		vTaskDelay( 1 );
		//vTaskDelayUntil
	}

	delete_publish_queues();
*/	

	RCCHECK(rcl_publisher_fini(&log_info_publisher, &node));
	
	//RCCHECK(rcl_publisher_fini(&wheel_radps_left_publisher, &node));
	//RCCHECK(rcl_publisher_fini(&wheel_radps_right_publisher, &node));
	RCCHECK(rcl_publisher_fini(&position_publisher, &node));
	RCCHECK(rcl_publisher_fini(&distance_publisher, &node));
		
	RCCHECK(rcl_subscription_fini(&subscriber_wheel_power_left, &node));
	RCCHECK(rcl_subscription_fini(&subscriber_wheel_power_right, &node));
	RCCHECK(rcl_ros_clock_fini(&ros_clock));

	RCCHECK(rcl_node_fini(&node));
	vTaskDelete(NULL);
}

extern "C"  void microros_interface_init(HadabotHW* _pHadabotHW) {
	pHadabotHW = _pHadabotHW;
    // start microROS task
    xTaskCreate(appMain, "uros_task", CONFIG_MICRO_ROS_APP_STACK, NULL, 5, NULL);	
}