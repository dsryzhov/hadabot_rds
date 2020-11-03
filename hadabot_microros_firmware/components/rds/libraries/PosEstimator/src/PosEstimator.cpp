#include "PosEstimator.h"

#define PI 3.141596

PosEstimator::PosEstimator(float _wheel_radius_m, float _wheelbase_m) 
: wheel_radius_m{_wheel_radius_m}, wheelbase_m{_wheelbase_m}
{
    pos_update_time = 0;
	pos.x = pos.y = pos.theta = 0;    
}

void PosEstimator::init(RotationSensor* _pLeftWheelRotationSensor, RotationSensor* _pRightWheelRotationSensor, MPU6050* _pMpu ) {
    pLeftWheelRotationSensor = _pLeftWheelRotationSensor;
    pRightWheelRotationSensor = _pRightWheelRotationSensor;
    pMpu = _pMpu;
}

void PosEstimator::getPosition(Position& _pos) {
    _pos.x = pos.x;
    _pos.y = pos.y;
    _pos.theta = pos.theta;
}

void PosEstimator::positionUpdateCallback(double measure_time, double measure_delta_time) {
		
    double wav_l = pLeftWheelRotationSensor->getAngularVelocityAtTime(measure_time);
    double wav_r = pRightWheelRotationSensor->getAngularVelocityAtTime(measure_time);


    double dt_s = measure_time - pos_update_time;

    if (dt_s > measure_delta_time) dt_s = measure_delta_time;

    updatePosition(wav_l, wav_r, dt_s);

	pos_update_time = measure_time;

}

bool PosEstimator::updateMpuAngles()
{

        mpuIntStatus = pMpu->getIntStatus();
		// get current FIFO count
		fifoCount = pMpu->getFIFOCount();

	    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
	        // reset so we can continue cleanly
	        pMpu->resetFIFO();

	    // otherwise, check for DMP data ready interrupt frequently)
	    } else if (mpuIntStatus & 0x02) {
	        // wait for correct available data length, should be a VERY short wait
	        while (fifoCount < packetSize) fifoCount = pMpu->getFIFOCount();

	        // read a packet from FIFO

	        pMpu->getFIFOBytes(fifoBuffer, packetSize);
	 		pMpu->dmpGetQuaternion(&q, fifoBuffer);
			pMpu->dmpGetGravity(&gravity, &q);
			pMpu->dmpGetYawPitchRoll(ypr, &q, &gravity);
			//printf("YAW: %3.1f, ", ypr[0] * 180/M_PI);
			//printf("PITCH: %3.1f, ", ypr[1] * 180/M_PI);
			//printf("ROLL: %3.1f \n", ypr[2] * 180/M_PI);
            return true;
        } 
        return false;
}        

void PosEstimator::updatePosition(double wav_l, double wav_r, double dt_s) {
    updateMpuAngles();
	
 	float d_left_m = (wav_l * dt_s * wheel_radius_m);
    float d_right_m = (wav_r * dt_s * wheel_radius_m);

    double d_center_m = (d_right_m + d_left_m) / 2.0;
    double phi_rad = (d_right_m - d_left_m) / wheelbase_m;

    double x_m_dt = d_center_m * std::cos(pos.theta);
    double y_m_dt = d_center_m * std::sin(pos.theta);
    double theta_rad_dt = phi_rad;

    pos.x += x_m_dt;
    pos.y += y_m_dt;
	//pos.theta += theta_rad_dt;

    pos.theta = ypr[0];

    if (pos.theta > PI) pos.theta = pos.theta - 2*PI ;
    else 
        if (pos.theta < -PI) pos.theta = pos.theta + 2*PI;
}