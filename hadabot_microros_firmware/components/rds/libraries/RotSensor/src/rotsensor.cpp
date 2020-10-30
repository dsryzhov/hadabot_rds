#include "rotsensor.h"
#include "driver/gpio.h"


#define TIMER_DIVIDER         16  //  Hardware timer clock divider
#define ROT_SENSOR_DISK_HOLES 20

#define ESP_INTR_FLAG_DEFAULT 0

/*
TODO
- configure count of holes
*/

static void IRAM_ATTR rotation_sensor_gpio_isr_handler(void* arg)
{
	RotationSensor* sensor = static_cast<RotationSensor*>(arg);

	portMUX_TYPE timerMux = sensor->getTimerMux();
	portENTER_CRITICAL_ISR(&timerMux);	

	int sensor_level = sensor->getSensorLevel();

    uint32_t gpio_num = (uint32_t) sensor->getSensorPIN();
	//int new_sensor_level = gpio_get_level((gpio_num_t)gpio_num);		
	int new_sensor_level = digitalRead(gpio_num);		
	
	if (new_sensor_level != sensor_level ) {

		double cur_time_sec = timerReadSeconds(sensor->getHwTimer());
		double last_time_sec = sensor->getLastMeasuredTime();

		double delta_time_sec = cur_time_sec - last_time_sec;

		if (delta_time_sec > 0 ) {
			
		
			sensor->setLastMeasuredTime(cur_time_sec);
			sensor->setSensorLevel(new_sensor_level);
			

			xQueueHandle sensor_evt_queue = sensor->getSensorEvtQueue();	
			xQueueSendFromISR(sensor_evt_queue, &delta_time_sec, NULL);	
		}				
	}
	portEXIT_CRITICAL_ISR(&timerMux);
}

static void sensor_radsp_calc_task(void* arg)
{
	
	RotationSensor* sensor = static_cast<RotationSensor*>(arg);
	portMUX_TYPE timerMux = sensor->getTimerMux();
	uint32_t gpio_num = (uint32_t) sensor->getSensorPIN();

	xQueueHandle sensor_evt_queue = sensor->getSensorEvtQueue();
	Motor* pMotor = sensor->getMotor();
	EMotorState motor_state;
	float angular_velocity = 0;
	ERotSensorMode mode = sensor->getSensorMode();
	int disk_holes_count = sensor->getDiskHolesCount();
	double hole_width_ratio = sensor->getHoleWidthRatio()/100.0;

	const TickType_t xDelay = 1000 / portTICK_PERIOD_MS;
	bool flLeftMotor = false;
	if (strcmp(sensor->getName(), "LeftMotorRotationSensor") == 0) 
		flLeftMotor = true;

	double prev_delta_time_sec = 0;
    double delta_time_sec;
	double full_delta_time_sec;
	int sign;
	int sensor_level;
    for(;;) {
        if(xQueueReceive(sensor_evt_queue, &delta_time_sec, xDelay) == pdTRUE ) {
			portENTER_CRITICAL_ISR(&timerMux);
			sensor_level = sensor->getSensorLevel();			
			portEXIT_CRITICAL_ISR(&timerMux);			
			
			//if (flLeftMotor) printf("delta_time : %f \n", (float)delta_time_sec);				

			if (sensor_level == 1) {
				if (prev_delta_time_sec != 0) {
						full_delta_time_sec = delta_time_sec + prev_delta_time_sec;
						angular_velocity = 2*3.141596 / disk_holes_count / full_delta_time_sec;

						motor_state = pMotor->getMotorState();
						if (motor_state > 0) sign = 1;
						else 
							if (motor_state < 0) sign = -1;
							else sign = 0;
					
						angular_velocity = angular_velocity * sign;	
						
						//portENTER_CRITICAL_ISR(&timerMux);
						
						//portEXIT_CRITICAL_ISR(&timerMux);
						//int cur_sensor_level = gpio_get_level((gpio_num_t)gpio_num);							
						//if (flLeftMotor)
							//printf("delta_time : %f  radsp : %f\n", (float)full_delta_time_sec, (float)angular_velocity);				
							//printf("delta_time : %f  radsp : %f    level: %d     cur: %d \n", (float)full_delta_time_sec, (float)angular_velocity, sensor_level, cur_sensor_level);				

					
						prev_delta_time_sec = 0;
						sensor->setAngularVelocity(angular_velocity);
				} else {
					prev_delta_time_sec = delta_time_sec;	
					//if (flLeftMotor) printf("Here 2 \n");
				}
			}
			else {
				prev_delta_time_sec = delta_time_sec;
				//if (flLeftMotor) printf("Here 3 \n");
			}
			
        } else {
			//if (flLeftMotor) printf("Here 4 \n");
			motor_state = pMotor->getMotorState();
			if (motor_state == STOPING_FORWARD || motor_state == STOPING_BACKWARD) {
				sensor->getMotor()->evStoped();
				sensor->setAngularVelocity(0);
				prev_delta_time_sec = 0;
			}
		}
    }
}

void RotationSensor::getAngularVelocity(float *val, int *sec, unsigned int *nanosec)
{
	float res;
	double _time;
	portENTER_CRITICAL_ISR(&timerMux);
	res = angular_velocity;			
	_time = measured_time;
	portEXIT_CRITICAL_ISR(&timerMux);	
	(*val) = res;

	(*sec) = (int) _time;
	(*nanosec) = (int) ( (_time - (double) (*sec)) * 1000000000); 
}

void RotationSensor::setAngularVelocity(float _angular_velocity) {
	portENTER_CRITICAL_ISR(&timerMux);
	angular_velocity = _angular_velocity;	
	measured_time = last_measured_time;		
	portEXIT_CRITICAL_ISR(&timerMux);	
	if (dataUpdatedCallback != NULL) (*dataUpdatedCallback)(_angular_velocity);
}

RotationSensor::RotationSensor(char* _sensor_name, uint32_t _sensor_pin, uint8_t _timer_num, Motor* _pMotor, int _disk_holes_count, ERotSensorMode _mode, double _hole_width_ratio) : 
 sensor_name(_sensor_name), 
 sensor_pin(_sensor_pin), 
 timer_num(_timer_num), 
 pMotor{_pMotor}, 
 disk_holes_count(_disk_holes_count), 
 mode(_mode), 
 hole_width_ratio{_hole_width_ratio},
 dataUpdatedCallback(NULL)
 {
	 
	angular_velocity = 0;
	
	pinMode(sensor_pin, INPUT_PULLUP);
	//pinMode(sensor_pin, INPUT);
	last_measured_time = 0;	
	sensor_level = digitalRead(sensor_pin);
    timer = timerBegin(timer_num, TIMER_DIVIDER, true);	
	
    sensor_evt_queue = xQueueCreate(1, sizeof(double));
	sensorTask = NULL;

 
}

void RotationSensor::begin() {

	timerMux = portMUX_INITIALIZER_UNLOCKED;
    xTaskCreatePinnedToCore(sensor_radsp_calc_task, sensor_name, 6144, this, 6, &sensorTask, 0);
	//xTaskCreate(sensor_radsp_calc_task, sensor_name, 6144, this, 6, &sensorTask);

	attachInterruptArg(digitalPinToInterrupt(sensor_pin), rotation_sensor_gpio_isr_handler, this, CHANGE);

/*
	gpio_set_intr_type((gpio_num_t)sensor_pin, GPIO_INTR_ANYEDGE);
	gpio_set_direction((gpio_num_t)sensor_pin, GPIO_MODE_INPUT);
	gpio_set_pull_mode((gpio_num_t)sensor_pin, GPIO_PULLUP_PULLDOWN);

	gpio_isr_handler_add((gpio_num_t) sensor_pin, rotation_sensor_gpio_isr_handler, this);		
*/

}

RotationSensor::~RotationSensor() {
	timerEnd(timer);
	if (sensorTask != NULL) vTaskDelete(sensorTask);
}
	



