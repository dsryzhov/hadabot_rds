#ifndef __POS_ESTIMATOR
#define __POS_ESTIMATOR

#include "IPosEstimator.h"
#include "rotsensor.h"
#include "TinyMPU6050.h"

struct Position {
	float x, y, theta;
};

class PosEstimator : public IPosEstimator {
public:
    PosEstimator(float _wheel_radius_m, float _wheelbase_m);
    ~PosEstimator() {}; 

    void init(RotationSensor* _pLeftWheelRotationSensor, RotationSensor* _pRightWheelRotationSensor, MPU6050* _pMpu );

    virtual void positionUpdateCallback(double measure_time, double measure_delta_time);
    void updatePosition(double wav_l, double wav_r, double dt_s);

    void getPosition(Position& _pos);
    

protected:

    RotationSensor* pLeftWheelRotationSensor;
    RotationSensor* pRightWheelRotationSensor;
    MPU6050* pMpu;

	float wheel_radius_m;
	float wheelbase_m;

	Position pos;

	double pos_update_time;

};

#endif