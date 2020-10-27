#include "Arduino.h"
#include "hcsr04.h"


static void IRAM_ATTR hcsr04_gpio_isr_handler(void* arg)
{
	HCSR04* sensor = static_cast<HCSR04*>(arg);
	
    uint32_t gpio_num = (uint32_t) sensor->getEchoPIN();
	
    int sensor_level = digitalRead(gpio_num);	

    if (sensor_level == 1) {
        sensor->setPulseStart(micros());
    } else {
        long int pulseStart = sensor->getPulseStart();
        if (pulseStart != -1) {
            long int duration = micros() - pulseStart; 
            int distance = (duration / 2) * 0.332;
            pulseStart = -1;
            sensor->setDistance(distance);
        }
    }

	
}

static void sensor_distance_trigger_task(void* arg)
{
	HCSR04* sensor = static_cast<HCSR04*>(arg);
    while (1) {
        sensor->sendTrigger();
        delay(60);
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
    distance = -1;
    pulseStart = -1;

    pinMode(_trigger, OUTPUT);
    pinMode(_echo, INPUT);
    digitalWrite(_trigger, LOW);  


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



