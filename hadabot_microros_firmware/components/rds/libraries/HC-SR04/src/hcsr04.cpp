#include "Arduino.h"
#include "hcsr04.h"


static void IRAM_ATTR hcsr04_gpio_isr_handler(void* arg)
{
	HCSR04* sensor = static_cast<HCSR04*>(arg);
    xQueueHandle sensor_evt_queue = sensor->getSensorEvtQueue();
	
    uint32_t gpio_num = (uint32_t) sensor->getEchoPIN();
	
    int sensor_level = sensor->getSensorLevel();
    int new_sensor_level = digitalRead(gpio_num);	

    if (new_sensor_level != sensor_level) {
        sensor->setSensorLevel(new_sensor_level);
        if (new_sensor_level == 1) {
            sensor->setPulseStart(micros());
        } else {
            long int pulseStart = sensor->getPulseStart();
            if (pulseStart != -1) {
                long int duration_mcs = micros() - pulseStart; 
                pulseStart = -1;            
                xQueueSendFromISR(sensor_evt_queue, &duration_mcs, NULL);
            }
        }
    }
}

static void sensor_distance_trigger_task(void* arg)
{
	HCSR04* sensor = static_cast<HCSR04*>(arg);
    xQueueHandle sensor_evt_queue = sensor->getSensorEvtQueue();
    long int duration_mcs;
    int distance_mm;

    const TickType_t xDelay = 60 / portTICK_PERIOD_MS;

    while (1) {
        sensor->sendTrigger();
        if(xQueueReceive(sensor_evt_queue, &duration_mcs, xDelay) == pdTRUE) {        
            distance_mm = (duration_mcs / 2) * 0.332; 
            sensor->setDistance(distance_mm);
        } else {
            distance_mm = -1; 
            sensor->setDistance(distance_mm);
        }
        //vTaskDelay( xDelay );
        delay(100);
    }
}


HCSR04::HCSR04(int trigger, int echo, int minRange, int maxRange)
{
    pinMode(trigger, OUTPUT);
    pinMode(echo, INPUT);
    _trigger = trigger;
    _echo = echo;
	_minRange = minRange;
    _maxRange = maxRange;

    distanceMeasuredCallback = NULL;
    distance = -1;
    pulseStart = -1;

    pinMode(_trigger, OUTPUT);
    pinMode(_echo, INPUT_PULLUP);
    digitalWrite(_trigger, LOW);  

    sensor_evt_queue = xQueueCreate(1, sizeof(long int));
    sensorTask = NULL;
}

HCSR04::~HCSR04() {
    if (sensorTask != NULL) vTaskDelete(sensorTask);
}

void HCSR04::begin() {
    mutex = portMUX_INITIALIZER_UNLOCKED;
    //xTaskCreate(sensor_distance_trigger_task, "distance_trigger", 6144, this, 6, sensorTask);
    xTaskCreatePinnedToCore(sensor_distance_trigger_task, "distance_trigger", 6144, this, 6, &sensorTask, 1);

    // temporarily
    //attachInterruptArg(_echo, hcsr04_gpio_isr_handler, this, CHANGE);    

	gpio_set_intr_type((gpio_num_t)_echo, GPIO_INTR_ANYEDGE);
	gpio_set_direction((gpio_num_t)_echo, GPIO_MODE_INPUT);
	gpio_set_pull_mode((gpio_num_t)_echo, GPIO_PULLUP_PULLDOWN);

	gpio_isr_handler_add((gpio_num_t) _echo, hcsr04_gpio_isr_handler, this);		
}

void HCSR04::setDistance(int _distance)
{
	portENTER_CRITICAL_ISR(&mutex);
	distance = _distance; 
	portEXIT_CRITICAL_ISR(&mutex);	
    if (distanceMeasuredCallback != NULL) 
        (*distanceMeasuredCallback)(_distance);
}

int HCSR04::getDistanceInMillimeters() {
    int res;
	portENTER_CRITICAL_ISR(&mutex);
	res = distance; 
	portEXIT_CRITICAL_ISR(&mutex);	
    return res;
};

void HCSR04::sendTrigger()
{
    digitalWrite(_trigger, HIGH);
    delayMicroseconds(10);
    digitalWrite(_trigger, LOW);  
}



