#ifndef __POS_ESTIMATOR
#define __POS_ESTIMATOR

#include "IPosEstimator.h"
#include "rotsensor.h"

struct Position {
	float x, y, theta;
};

class PosEstimator : public IPosEstimator {
public:
    PosEstimator(float _wheel_radius_m, float _wheelbase_m);
    ~PosEstimator() {}; 

    void init(RotationSensor* _pLeftWheelRotationSensor, RotationSensor* _pRightWheelRotationSensor);

    virtual void positionUpdateCallback(double measure_time, double measure_delta_time);
    void updatePosition(double wav_l, double wav_r, double dt_s);

    void getLastPostion(Position* pPos);
    

protected:

    RotationSensor* pLeftWheelRotationSensor;
    RotationSensor* pRightWheelRotationSensor;

	float wheel_radius_m;
	float wheelbase_m;

	Position pos;

	double pos_update_time;

};

#endif