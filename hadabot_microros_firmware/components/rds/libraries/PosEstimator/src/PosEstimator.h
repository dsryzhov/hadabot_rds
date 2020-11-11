#ifndef __POS_ESTIMATOR
#define __POS_ESTIMATOR

#include "IPosEstimator.h"
#include "rotsensor.h"
//#include "TinyMPU6050.h"
#include <Kalman.h>
#include "MPU6050.h"
#include "IPosController.h"
#include "Geometry.h"


class PosEstimator : public IPosEstimator {
public:
    PosEstimator(float _wheel_radius_m, float _wheelbase_m);
    ~PosEstimator() {}; 

    void init(RotationSensor* _pLeftWheelRotationSensor, RotationSensor* _pRightWheelRotationSensor, MPU6050* _pMpu, IPosController* _pPosController );

    virtual void positionUpdateCallback(double measure_time, double measure_delta_time);
    bool updatePosition(double wav_l, double wav_r, double dt_s);

    void getPosition(Position& _pos);
    void getTwist(Twist& _twist);
    void getQuaternion(Quaternion _q);

    bool updateMpuAngles();

    float getMpuYaw() {return ypr[0];}

    void begin();
    

protected:

    RotationSensor* pLeftWheelRotationSensor;
    RotationSensor* pRightWheelRotationSensor;
    MPU6050* pMpu;
    IPosController* pPosController;

	float wheel_radius_m;
	float wheelbase_m;

	Position pos;
    Twist twist;

	double pos_update_time;

    Quaternion q;           // [w, x, y, z]         quaternion container
    VectorFloat gravity;    // [x, y, z]            gravity vector
    float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
    int32_t gyro_data[3];

    int16_t ax, ay, az;
    int16_t gx, gy, gz;


    uint16_t packetSize = 42;    // expected DMP packet size (default is 42 bytes)
    uint16_t fifoCount;     // count of all bytes currently in FIFO
    uint8_t fifoBuffer[64]; // FIFO storage buffer
    uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU

    float w_theta;

    Kalman kalmanTheta;
    Kalman kalmanX;
    Kalman kalmanY;

};

#endif