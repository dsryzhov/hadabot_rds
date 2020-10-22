#include "sensor.h"


#define TIMER_DIVIDER         16  //  Hardware timer clock divider

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
		
		if (delta_time_sec > 0.005 && sensor_level != new_sensor_level) {
			xQueueSendFromISR(sensor_evt_queue, &delta_time_sec, NULL);		
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
	
    double delta_time_sec;
    for(;;) {
        if(xQueueReceive(sensor_evt_queue, &delta_time_sec, portMAX_DELAY)) {
			if (delta_time_sec > 0.0) {
				
				int sensor_level = sensor->getSensorLevel();
				
				angular_velocity = 2*3.141596 / 20.0 / delta_time_sec;
				
				// текущий уровень сигнала сенсора - высокий, т.е. отверстие. Значит прошедший соответствует перегородке, которая шире отверстия.
				if (sensor_level == 1) angular_velocity*=13.0/21.0;  
				else angular_velocity *= 8.0/21.0;

				int sign = 1;
				motor_state = pMotor->getMotorState();
				
				if (motor_state < 0) sign = -1;
			
				if (motor_state != STOPED)
					angular_velocity = angular_velocity * sign;	
				else angular_velocity = 0;
				
				printf("delta_time : %f  radsp : %f\n", (float)delta_time_sec, (float)angular_velocity);				
			} else angular_velocity = 0;
			sensor->setAngularVelocity(angular_velocity);
        }
    }
}


RotationSensor::RotationSensor(char* _sensor_name, uint32_t _sensor_pin, uint8_t _timer_num, Motor* _pMotor) : 
 sensor_name(_sensor_name), sensor_pin(_sensor_pin), timer_num(_timer_num), pMotor{_pMotor} {
	 
	angular_velocity = 0;
	timerMux = portMUX_INITIALIZER_UNLOCKED;
	pinMode(sensor_pin, INPUT_PULLUP);

	last_measured_time = 0;	
	sensor_level = digitalRead(sensor_pin);
    timer = timerBegin(timer_num, TIMER_DIVIDER, true);	
	
    sensor_evt_queue = xQueueCreate(10, sizeof(double));
    xTaskCreate(sensor_radsp_calc_task, sensor_name, 6144, this, 10, NULL);

	attachInterruptArg(sensor_pin, rotation_sensor_gpio_isr_handler, this, CHANGE); 
}

RotationSensor::~RotationSensor() {
	timerEnd(timer);
}
	




	
/*

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
				
				float angular_velocity = 2*3.141596 / 20.0 / delta_time_sec;
				if (left_wheel_sensor_state == true) angular_velocity*=13.0/21.0;  // текущий уровень сигнала сенсора - высокий, т.е. отверстие. Значит прошедший соответствует перегородке, которая шире отверстия.
				else angular_velocity *= 8.0/21.0;


				int sign = 1;
				if (left_wheel_state < 0) sign = -1;
			
				if (left_wheel_state != STOPED)
					angular_velocity = angular_velocity * sign;	
				else angular_velocity = 0;
				
				//if (angular_velocity < 20)
					wheel_radps_left_msg.data = angular_velocity;
				
				//printf("delta_time : %f  radsp : %f\n", (float)delta_time_sec, (float)angular_velocity);				

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
				float angular_velocity = 2*3.141596 / 20.0 / delta_time_sec;
				if (right_wheel_sensor_state == true) angular_velocity*=123.0/210.0;  // текущий уровень сигнала сенсора - высокий, т.е. отверстие. Значит прошедший соответствует перегородке, которая шире отверстия.
				else angular_velocity *= 87.0/210.0;

				//printf("delta_time : %f  radsp : %f\n", (float)delta_time_sec, (float)angular_velocity);
				
				int sign = 1;
				if (right_wheel_state < 0) sign = -1;
				
				if (right_wheel_state != STOPED)
				   angular_velocity = angular_velocity * sign;	
			

				//if (angular_velocity < 20)
					wheel_radps_right_msg.data = angular_velocity;

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


*/


