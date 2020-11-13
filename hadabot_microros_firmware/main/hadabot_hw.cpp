#include "hadabot_hw.h"
#include "sdkconfig.h"
#include "Arduino.h"
#include "MPU6050_6Axis_MotionApps20.h"

// use 5000 Hz as a motor contorl base frequency
#define MOTOR_CONTROL_BASE_FREQ     5000
#define LEFT_MOTOR_FORWARD_CHANNEL 0
#define LEFT_MOTOR_BACKWARD_CHANNEL 2
#define RIGHT_MOTOR_FORWARD_CHANNEL 1
#define RIGHT_MOTOR_BACKWARD_CHANNEL 3

#define WHEELS_BASE 0.117
#define WHEEL_RADIUS 0.032
#define WHEEL_MAX_ANGULAR_VELOCITY 21
#define DESIRED_LINEAR_VELOCITY 0.5
#define DESIRED_ANGULAR_VELOCITY 10

#define LEFT_ROTATION_SENSOR_TIMER_NUM   0
#define RIGHT_ROTATION_SENSOR_TIMER_NUM   1

#define GOAL_ANGLE_THRESH_RAD 0.3
#define POS_ANGLE_THRESH_RAD 0.3
#define DISTANCE_THRESH_M  0.02 
#define POS_CONTROL_PERIOD_MS 20
              

#define PIN_SDA 21
#define PIN_CLK 22
#define MPU_INTERRUPT_PIN 4

void task_initI2C(void *ignore) {
	i2c_config_t conf;
	conf.mode = I2C_MODE_MASTER;
	conf.sda_io_num = (gpio_num_t)PIN_SDA;
	conf.scl_io_num = (gpio_num_t)PIN_CLK;
	conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
	conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
	conf.master.clk_speed = 400000;
	ESP_ERROR_CHECK(i2c_param_config(I2C_NUM_0, &conf));
	ESP_ERROR_CHECK(i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0));
	vTaskDelete(NULL);
}

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady()
{
	mpuInterrupt = true;
}

extern "C" bool isDmpDataReady() {
	if (mpuInterrupt) {
		mpuInterrupt = false;
		return true;
	} return false;
}
 


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
		   CONFIG_HADABOT_FW_SONAR_ECHO_PIN, 20, 4000),
	pos_estimator(WHEEL_RADIUS, WHEELS_BASE)

{
//			  uint32_t flags = ESP_INTR_FLAG_EDGE |//< Edge-triggered interrupt
//    ESP_INTR_FLAG_IRAM; //< ISR can be called if cache is disabled


	pos_estimator.init(&leftWheelRotationSensor, &rightWheelRotationSensor, &mpu, &pos_controller);

	leftWheelRotationSensor.setPositionUpdateCallback(&pos_estimator);
	rightWheelRotationSensor.setPositionUpdateCallback(&pos_estimator);

	motion_controller.init(&leftMotor, &rightMotor, &leftWheelRotationSensor, &rightWheelRotationSensor);//0.143852, 21);
	motion_controller.setParams(WHEEL_RADIUS, WHEELS_BASE, WHEEL_MAX_ANGULAR_VELOCITY);

	pos_controller.init(&motion_controller);

	pos_controller.setParams(DESIRED_LINEAR_VELOCITY, 
								DESIRED_ANGULAR_VELOCITY, 
								DISTANCE_THRESH_M, 
								POS_ANGLE_THRESH_RAD, 
								GOAL_ANGLE_THRESH_RAD, 
								POS_CONTROL_PERIOD_MS);
}

HadabotHW::~HadabotHW() {
}


void HadabotHW::begin() {
	
//	uint32_t flags = ESP_INTR_FLAG_EDGE | ESP_INTR_FLAG_IRAM;
//	gpio_install_isr_service(flags);
	leftWheelRotationSensor.begin();
	rightWheelRotationSensor.begin();
	//hcsr04.begin();

    xTaskCreate(&task_initI2C, "mpu_task", 2048, NULL, 6, NULL);
    vTaskDelay(500/portTICK_PERIOD_MS);


	mpu.initialize();

	mpu.dmpInitialize();

	// This need to be setup individually
	mpu.setXGyroOffset(220);
	mpu.setYGyroOffset(76);
	mpu.setZGyroOffset(-85);
	mpu.setZAccelOffset(1788);

	mpu.CalibrateAccel(6);
	mpu.CalibrateGyro(6);
	//mpu.PrintActiveOffsets();
	
	mpu.setDMPEnabled(true);

	printf("=========== Mpu rate %d\n", (int)mpu.getRate()) ;

	attachInterrupt(digitalPinToInterrupt(MPU_INTERRUPT_PIN), dmpDataReady, RISING);
	
	vTaskDelay(500/portTICK_PERIOD_MS);

	pos_estimator.begin();
	motion_controller.begin();
	
	printf("Hadabot Hw started.\n");
}



