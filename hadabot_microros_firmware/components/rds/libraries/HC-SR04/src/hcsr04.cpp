#include "Arduino.h"
#include "hcsr04.h"


static void IRAM_ATTR hcsr04_gpio_isr_handler(void* arg)
{
	HCSR04* sensor = static_cast<HCSR04*>(arg);
    xQueueHandle sensor_evt_queue = sensor->getSensorEvtQueue();
	
    uint32_t gpio_num = (uint32_t) sensor->getEchoPIN();
	
    int sensor_level = digitalRead(gpio_num);	

    if (sensor_level == 1) {
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

static void sensor_distance_trigger_task(void* arg)
{
	HCSR04* sensor = static_cast<HCSR04*>(arg);
    xQueueHandle sensor_evt_queue = sensor->getSensorEvtQueue();
    long int duration_mcs;
    int distance_mm;

    while (1) {
        sensor->sendTrigger();
        if(xQueueReceive(sensor_evt_queue, &duration_mcs, portMAX_DELAY)) {        
            distance_mm = (duration_mcs / 2) * 0.332;
            sensor->setDistance(distance_mm);
        }
        delay(10);
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
    pinMode(_echo, INPUT);
    digitalWrite(_trigger, LOW);  

    sensor_evt_queue = xQueueCreate(1, sizeof(long int));

    xTaskCreate(sensor_distance_trigger_task, "distance_trigger", 6144, this, 10, NULL);

    attachInterruptArg(_echo, hcsr04_gpio_isr_handler, this, CHANGE);    
}



void HCSR04::sendTrigger()
{

//    digitalWrite(_trigger, LOW);
//    delayMicroseconds(5);
    digitalWrite(_trigger, HIGH);
    delayMicroseconds(10);
    digitalWrite(_trigger, LOW);  
}



