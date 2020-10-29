#include "hadabot_hw.h"

#include "sdkconfig.h"

// use 5000 Hz as a motor contorl base frequency
#define MOTOR_CONTROL_BASE_FREQ     5000
#define LEFT_MOTOR_FORWARD_CHANNEL 0
#define LEFT_MOTOR_BACKWARD_CHANNEL 2
#define RIGHT_MOTOR_FORWARD_CHANNEL 1
#define RIGHT_MOTOR_BACKWARD_CHANNEL 3


#define LEFT_ROTATION_SENSOR_TIMER_NUM   0
#define RIGHT_ROTATION_SENSOR_TIMER_NUM   1

/*
TODO
- configure hadabot sensors via sdkconfig
*/

HadabotHW::HadabotHW() :
  leftMotor("Left motor",
	  		LEFT_MOTOR_FORWARD_CHANNEL, 
			LEFT_MOTOR_BACKWARD_CHANNEL, 
			CONFIG_HADABOT_LEFT_MOTOR_FORWARD_PIN, 
			CONFIG_HADABOT_LEFT_MOTOR_BACKWARD_PIN, 
			MOTOR_CONTROL_BASE_FREQ),
  rightMotor("Right motor",
	  		RIGHT_MOTOR_FORWARD_CHANNEL, 
			RIGHT_MOTOR_BACKWARD_CHANNEL, 
			CONFIG_HADABOT_RIGHT_MOTOR_FORWARD_PIN, 
			CONFIG_HADABOT_RIGHT_MOTOR_BACKWARD_PIN, 
			MOTOR_CONTROL_BASE_FREQ),
  leftWheelRotationSensor("LeftMotorRotationSensor", 
		CONFIG_HADABOT_LEFT_WHEEL_ROT_SENSOR_PIN, 
		LEFT_ROTATION_SENSOR_TIMER_NUM, 
		&leftMotor, 
		20,
		FRONT_HIGH,
		CONFIG_HADABOT_LEFT_ROT_SENSOR_HOLE_WIDTH_RATIO),
  rightWheelRotationSensor("RightMotorRotationSensor", 
		CONFIG_HADABOT_RIGHT_WHEEL_ROT_SENSOR_PIN, 
		RIGHT_ROTATION_SENSOR_TIMER_NUM, 
		&rightMotor,
		20,
		FRONT_HIGH,
		CONFIG_HADABOT_RIGHT_ROT_SENSOR_HOLE_WIDTH_RATIO),
	hcsr04(CONFIG_HADABOT_FW_SONAR_TRIG_PIN, 
		   CONFIG_HADABOT_FW_SONAR_ECHO_PIN, 20, 4000)
{
//			  uint32_t flags = ESP_INTR_FLAG_EDGE |//< Edge-triggered interrupt
//    ESP_INTR_FLAG_IRAM; //< ISR can be called if cache is disabled


}

HadabotHW::~HadabotHW() {
}

void HadabotHW::begin() {
	
	uint32_t flags = ESP_INTR_FLAG_EDGE | ESP_INTR_FLAG_IRAM;

	gpio_install_isr_service(flags);
	leftWheelRotationSensor.begin();
	rightWheelRotationSensor.begin();
	hcsr04.begin();
	
	printf("Hadabot Hw started.\n");
}

