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
		   CONFIG_HADABOT_FW_SONAR_ECHO_PIN, 20, 4000),
	pos_estimator(0.032, 0.117)

{
//			  uint32_t flags = ESP_INTR_FLAG_EDGE |//< Edge-triggered interrupt
//    ESP_INTR_FLAG_IRAM; //< ISR can be called if cache is disabled


	pos_estimator.init(&leftWheelRotationSensor, &rightWheelRotationSensor, &mpu);

	leftWheelRotationSensor.setPositionUpdateCallback(&pos_estimator);
	rightWheelRotationSensor.setPositionUpdateCallback(&pos_estimator);

}

HadabotHW::~HadabotHW() {
}

#define PIN_SDA 21
#define PIN_CLK 22

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

	mpu.setDMPEnabled(true);


//	mpu.Calibrate();
	
	printf("Hadabot Hw started.\n");
}



