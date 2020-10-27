#ifndef HCSR04_h
#define HCSR04_h

#include "Arduino.h"

class HCSR04
{
  public:
	  HCSR04(int trigger, int echo, int minRange, int maxRange);

    inline int getDistanceInMillimeters() {return distance;};
    
    void sendTrigger();
    inline void setDistance(int _distance) {distance = _distance; }
    inline int getEchoPIN() {return _echo;}
    inline void setPulseStart(int _pulseStart) {pulseStart = _pulseStart;}
    inline long int getPulseStart() {return pulseStart;}


  private:
    int _trigger;
    int _echo;
	int _minRange = -1;
	int _maxRange = -1;
  int distance;
  long int pulseStart;
};

#endif