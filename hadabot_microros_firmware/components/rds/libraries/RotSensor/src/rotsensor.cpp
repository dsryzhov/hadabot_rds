#include "rotsensor.h"


#define TIMER_DIVIDER         16  //  Hardware timer clock divider
#define ROT_SENSOR_DISK_HOLES 20

/*
TODO
- set sensor calibration coefficients
- configure min time between sensor signal changes
- configure count of holes
*/

static void IRAM_ATTR rotation_sensor_gpio_isr_handler(void* arg)
{
	RotationSensor* sensor = static_cast<RotationSensor*>(arg);
	
	portMUX_TYPE timerMux = sensor->getTimerMux();
	
	portENTER_CRITICAL_ISR(&timerMux);
	

    uint32_t gpio_num = (uint32_t) sensor->getSensorPIN();
	xQueueHandle sensor_evt_queue = sensor->getSensorEvtQueue();
	
	
	int sensor_level = sensor->getSensorLevel();
	double last_time_sec = sensor->getLastMeasuredTime();
	
	int new_sensor_level = digitalRead(gpio_num);
	
//	if (new_sensor_level != sensor_level) {
		

		double cur_time_sec = timerReadSeconds(sensor->getHwTimer());
		
		double delta_time_sec = cur_time_sec - last_time_sec;
		
		ERotSensorMode mode = sensor->getSensorMode();
		
		if (mode == FRONT_CHANGE) {
			if (delta_time_sec > 0.005 && sensor_level != new_sensor_level) {
				xQueueSendFromISR(sensor_evt_queue, &delta_time_sec, NULL);		
			}
		}
		else 
			if (mode == FRONT_HIGH) {
			}
		
		
		sensor->setLastMeasuredTime(cur_time_sec);
		sensor->setSensorLevel(new_sensor_level);
		
		portEXIT_CRITICAL_ISR(&timerMux);
		
		
//	}
}

static void sensor_radsp_calc_task(void* arg)
{
	
	RotationSensor* sensor = static_cast<RotationSensor*>(arg);
	
	xQueueHandle sensor_evt_queue = sensor->getSensorEvtQueue();
	Motor* pMotor = sensor->getMotor();
	EMotorState motor_state;
	float angular_velocity = 0;
	ERotSensorMode mode = sensor->getSensorMode();
	int disk_holes_count = sensor->getDiskHolesCount();
	double hole_width_ratio = sensor->getHoleWidthRatio()/100.0;
	
    double delta_time_sec;
    for(;;) {
        if(xQueueReceive(sensor_evt_queue, &delta_time_sec, portMAX_DELAY)) {
			if (delta_time_sec > 0.0) {
				
				int sensor_level = sensor->getSensorLevel();
				
				angular_velocity = 2*3.141596 / disk_holes_count / delta_time_sec;
				
				if (mode == FRONT_CHANGE) {
						
					// текущий уровень сигнала сенсора - высокий, т.е. отверстие. Значит прошедший соответствует перегородке, которая шире отверстия.
					if (sensor_level == 1) angular_velocity *= hole_width_ratio;//13.0/21.0;  
					else angular_velocity *= 1.0-hole_width_ratio; //8.0/21.0;
				}

				int sign = 1;
				motor_state = pMotor->getMotorState();
				if (motor_state < 0) sign = -1;
			
				if (motor_state != STOPED)
					angular_velocity = angular_velocity * sign;	
				else angular_velocity = 0;
				
				//printf("delta_time : %f  radsp : %f\n", (float)delta_time_sec, (float)angular_velocity);				
			} else angular_velocity = 0;
			sensor->setAngularVelocity(angular_velocity);
        }
    }
}


RotationSensor::RotationSensor(char* _sensor_name, uint32_t _sensor_pin, uint8_t _timer_num, Motor* _pMotor, int _disk_holes_count, ERotSensorMode _mode, double _hole_width_ratio) : 
 sensor_name(_sensor_name), 
 sensor_pin(_sensor_pin), 
 timer_num(_timer_num), 
 pMotor{_pMotor}, 
 disk_holes_count(_disk_holes_count), 
 mode(_mode), 
 hole_width_ratio{_hole_width_ratio} 
 {
	 
	angular_velocity = 0;
	timerMux = portMUX_INITIALIZER_UNLOCKED;
	pinMode(sensor_pin, INPUT_PULLUP);

	last_measured_time = 0;	
	sensor_level = digitalRead(sensor_pin);
    timer = timerBegin(timer_num, TIMER_DIVIDER, true);	
	
    sensor_evt_queue = xQueueCreate(10, sizeof(double));
    xTaskCreate(sensor_radsp_calc_task, sensor_name, 6144, this, 10, NULL);

	if (mode == FRONT_CHANGE)
		attachInterruptArg(sensor_pin, rotation_sensor_gpio_isr_handler, this, CHANGE);
	else 
		if (mode == FRONT_HIGH)
			attachInterruptArg(sensor_pin, rotation_sensor_gpio_isr_handler, this, HIGH);
}

RotationSensor::~RotationSensor() {
	timerEnd(timer);
}
	



