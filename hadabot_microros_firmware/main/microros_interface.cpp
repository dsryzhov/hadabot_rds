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
#include <nav_msgs/msg/odometry.h>
#include <geometry_msgs/msg/twist.h>
#include <geometry_msgs/msg/pose2_d.h>
#include <hadabot_msgs/msg/odom2_d.h>
#include <stdio.h>
#include <unistd.h>
#include <time.h>
#include "Geometry.h"

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

rcl_subscription_t subscriber_hadabot_cmd_vel;
rcl_subscription_t subscriber_cmd_vel;
geometry_msgs__msg__Twist cmd_vel_msg;

rcl_subscription_t subscriber_hadabot_goal_pos;
geometry_msgs__msg__Pose2D goal_pos_msg;

rcl_publisher_t wheel_radps_left_publisher;
rcl_publisher_t wheel_radps_right_publisher;

sensor_msgs__msg__Temperature wheel_radps_left_msg;
sensor_msgs__msg__Temperature wheel_radps_right_msg;

hadabot_msgs__msg__Odom2D odom2d_msg;
rcl_publisher_t odom_publisher;

//std_msgs__msg__Float32 wheel_radps_left_msg;
//std_msgs__msg__Float32 wheel_radps_right_msg;


rcl_publisher_t distance_publisher;
//std_msgs__msg__Float32 distance_msg;
sensor_msgs__msg__Temperature distance_msg;


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
		//double time_sec =  base_time_point_value / 1000000000.0;

		long unsigned int time_sec =  (long unsigned int) (base_time_point_value / 1000000000.0);
		long unsigned int time_nsec = (long unsigned int) (base_time_point_value - time_sec*1000000000);


		sprintf(log_info_msg.data.data, "Hadabot heartbeat sec: %lu  nsec: %lu", time_sec, time_nsec);


	 //fprintf(stdout, "Cur time ns %llu, msg time ns %llu, difference %lld / %ld\n",
    //    curtime, msgtime, duration_ns, duration_ms);		

		//sprintf(log_info_msg.data.data, "Hadabot heartbeat");
		log_info_msg.data.size = strlen(log_info_msg.data.data);

		rcl_publish(&log_info_publisher, (const void*)&log_info_msg, NULL);
		printf(log_info_msg.data.data);
		printf("\n");
	}
}

void goal_pos_callback(const void * msgin) {

	const geometry_msgs__msg__Pose2D * msg = (const geometry_msgs__msg__Pose2D *)msgin;

	Position goal_pos(msg->x, msg->y, msg->theta);
	printf("Received goal position x: %f y: %f, theta: %f\n", goal_pos.x, goal_pos.y, goal_pos.theta);

	pHadabotHW->getPosController()->setGoalPosition(goal_pos);
}

void cmd_vel_callback(const void * msgin) {

	rcl_time_point_value_t base_time_point_value;
	rcl_clock_get_now(&ros_clock, &base_time_point_value);

	long unsigned int time_sec =  (long unsigned int) (base_time_point_value / 1000000000.0);
	long unsigned int time_nsec = (long unsigned int) (base_time_point_value - time_sec*1000000000);

	printf("Received cmd_vel at sec: %lu nsec: %lu \n", time_sec, time_nsec);

	const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;
	//printf("Received cmd_vel message");
	if (msg != NULL) {
		//printf("Data: %f\n", msg->data);
		pHadabotHW->getMotionController()->updateMotion(msg->linear.x, msg->angular.z);
	}
}


void sensor_data_publish_timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
	UNUSED(last_call_time);

	rcl_time_point_value_t base_time_point_value;
	rcl_clock_get_now(&ros_clock, &base_time_point_value);
	
	long unsigned int time_sec =  (long unsigned int) (base_time_point_value / 1000000000.0);
	long unsigned int time_nsec = (long unsigned int) (base_time_point_value - time_sec*1000000000);

	PosEstimator *pPosEstimator = pHadabotHW->getPosEstimator();
	Position pos;
	Twist twist;
	pPosEstimator->getPosition(pos);
	pPosEstimator->getTwist(twist);
	//printf("x: %f   y:   %f    theta   %f\n", pos.x, pos.y, pos.theta);

	odom2d_msg.x = pos.x;
	odom2d_msg.y = pos.y;
	odom2d_msg.theta = pos.theta;
	odom2d_msg.v = twist.v;
	odom2d_msg.w = twist.w;

			
	odom2d_msg.stamp.sec = time_sec;
	odom2d_msg.stamp.nanosec = time_nsec;

	rcl_publish(&odom_publisher, (const void*)&odom2d_msg, NULL);	
	

}



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
	odom_msg.header.frame_id.data = (char*)malloc(FRAME_ID_SIZE * sizeof(char));
	odom_msg.header.frame_id.size=0;
	odom_msg.header.frame_id.capacity=FRAME_ID_SIZE;	
	sprintf(odom_msg.header.frame_id.data, "FrameO");	
	odom_msg.header.frame_id.size = strlen(odom_msg.header.frame_id.data);	

	odom_msg.child_frame_id.data = (char*)malloc(FRAME_ID_SIZE * sizeof(char));
	odom_msg.child_frame_id.size=0;
	odom_msg.child_frame_id.capacity=FRAME_ID_SIZE;	
	sprintf(odom_msg.child_frame_id.data, "FrameC");	
	odom_msg.child_frame_id.size = strlen(odom_msg.child_frame_id.data);	
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
	RCCHECK(rclc_timer_init_default(&sensor_data_publish_timer, &support, RCL_MS_TO_NS(20), sensor_data_publish_timer_callback));

	RCCHECK(rclc_publisher_init_default(&odom_publisher, &node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(hadabot_msgs, msg, Odom2D), "hadabot/odom2d"));

/*	
	RCCHECK(rclc_publisher_init_default(&wheel_radps_left_publisher, &node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Temperature), "hadabot/wheel_radps_left_timestamped"));


	RCCHECK(rclc_publisher_init_default(&wheel_radps_right_publisher, &node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Temperature), "hadabot/wheel_radps_right_timestamped"));
*/

	RCCHECK(rclc_publisher_init_default(&distance_publisher, &node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Temperature), "hadabot/distance_forward_timestamped"));


	RCCHECK(rclc_subscription_init_default(
		&subscriber_cmd_vel,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
		"/cmd_vel"));

	RCCHECK(rclc_subscription_init_default(
		&subscriber_hadabot_cmd_vel,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
		"/hadabot/cmd_vel"));

	RCCHECK(rclc_subscription_init_default(
		&subscriber_hadabot_goal_pos,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Pose2D),
		"/hadabot/goal_pose"));



	// create executor
	rclc_executor_t executor = rclc_executor_get_zero_initialized_executor();
	RCCHECK(rclc_executor_init(&executor, &support.context, 5, &allocator));

	unsigned int rcl_wait_timeout = 1;   // timeout for waiting for new data from subscribed topics
	RCCHECK(rclc_executor_set_timeout(&executor, RCL_MS_TO_NS(rcl_wait_timeout)));

	RCCHECK(rclc_executor_add_timer(&executor, &sensor_data_publish_timer));

	RCCHECK(rclc_executor_add_subscription(&executor, &subscriber_hadabot_cmd_vel, &cmd_vel_msg, &cmd_vel_callback, ON_NEW_DATA));

	RCCHECK(rclc_executor_add_subscription(&executor, &subscriber_cmd_vel, &cmd_vel_msg, &cmd_vel_callback, ON_NEW_DATA));

		RCCHECK(rclc_executor_add_subscription(&executor, &subscriber_hadabot_goal_pos, &goal_pos_msg, &goal_pos_callback, ON_NEW_DATA));

	RCCHECK(rclc_executor_add_timer(&executor, &log_info_timer));	

	rclc_executor_spin(&executor);	

	RCCHECK(rcl_publisher_fini(&log_info_publisher, &node));
	
	//RCCHECK(rcl_publisher_fini(&wheel_radps_left_publisher, &node));
	//RCCHECK(rcl_publisher_fini(&wheel_radps_right_publisher, &node));
	RCCHECK(rcl_publisher_fini(&odom_publisher, &node));
	RCCHECK(rcl_publisher_fini(&distance_publisher, &node));
	RCCHECK(rcl_subscription_fini(&subscriber_cmd_vel, &node));
	RCCHECK(rcl_subscription_fini(&subscriber_hadabot_goal_pos, &node));

	RCCHECK(rcl_ros_clock_fini(&ros_clock));

	RCCHECK(rcl_node_fini(&node));
	vTaskDelete(NULL);
}

extern "C"  void microros_interface_init(HadabotHW* _pHadabotHW) {
	pHadabotHW = _pHadabotHW;
    // start microROS task
    xTaskCreate(appMain, "uros_task", CONFIG_MICRO_ROS_APP_STACK, NULL, 5, NULL);	
}