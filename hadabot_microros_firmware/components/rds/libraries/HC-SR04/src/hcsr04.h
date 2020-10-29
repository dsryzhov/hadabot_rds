#ifndef HCSR04_h
#define HCSR04_h

#include "Arduino.h"

class HCSR04
{
  public:
	  HCSR04(int trigger, int echo, int minRange, int maxRange);
    ~HCSR04();

    void begin();

    int getDistanceInMillimeters();

    inline void setDistanceMeasuredCallback(void (*_distanceMeasuredCallback)(int)) {distanceMeasuredCallback = _distanceMeasuredCallback;}
    
    void sendTrigger();
    void setDistance(int _distance);
    inline int getEchoPIN() {return _echo;}
    inline void setPulseStart(int _pulseStart) {pulseStart = _pulseStart;}
    inline long int getPulseStart() {return pulseStart;}
    inline xQueueHandle getSensorEvtQueue() {return sensor_evt_queue;}
    inline int getSensorLevel() {return sensorLevel; }
    inline void setSensorLevel(int _sensorLevel) {sensorLevel = _sensorLevel;}

  private:
    int _trigger;
    int _echo;
    int _minRange = -1;
    int _maxRange = -1;
    int distance;
    int sensorLevel;
    long int pulseStart;
    xQueueHandle sensor_evt_queue = NULL;
    void (*distanceMeasuredCallback)(int);
    portMUX_TYPE mutex;
    TaskHandle_t sensorTask;
};

#endif