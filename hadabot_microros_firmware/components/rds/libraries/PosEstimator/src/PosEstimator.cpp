#include "PosEstimator.h"

extern "C" bool isDmpDataReady();

PosEstimator::PosEstimator(float _wheel_radius_m, float _wheelbase_m) 
: wheel_radius_m{_wheel_radius_m}, wheelbase_m{_wheelbase_m}
{
    pos_update_time = 0;
	pos.x = pos.y = pos.theta = 0;    
    twist.v = twist.w = 0;
}

void PosEstimator::init(RotationSensor* _pLeftWheelRotationSensor, RotationSensor* _pRightWheelRotationSensor, MPU6050* _pMpu, IPosController* _pPosController ) {
    pLeftWheelRotationSensor = _pLeftWheelRotationSensor;
    pRightWheelRotationSensor = _pRightWheelRotationSensor;
    pMpu = _pMpu;
    pPosController = _pPosController;
}

void PosEstimator::begin() {
    updateMpuAngles();
    pos.theta = ypr[0];
    w_theta = pos.theta;
    kalmanTheta.setAngle(pos.theta);
    kalmanX.setAngle(0);
    kalmanY.setAngle(0);

}

void PosEstimator::getPosition(Position& _pos) {
    _pos.x = pos.x;
    _pos.y = pos.y;
    _pos.theta = pos.theta;
}

void PosEstimator::getQuaternion(Quaternion _q) {

}

void PosEstimator::getTwist(Twist& _twist) {
    _twist.v = twist.v;
    _twist.w = twist.w;
}



bool PosEstimator::updateMpuAngles()
{
    bool mpuInterrupt = isDmpDataReady();
    if (!mpuInterrupt && fifoCount < packetSize) return false;

  //  if (isDmpDataReady()) {

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

            //pMpu->dmpGetGyro(gyro_data, fifoBuffer);

            fifoCount -= packetSize;


            pMpu->getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

			//printf("YAW: %3.1f, ", ypr[0] * 180/M_PI);
			//printf("PITCH: %3.1f, ", ypr[1] * 180/M_PI);
			//printf("ROLL: %3.1f \n", ypr[2] * 180/M_PI);
            return true;
        } 
        return false;
    //}

/*
    if (isDmpDataReady()) {
            fifoCount = pMpu->getFIFOCount();
            while (fifoCount < packetSize) fifoCount = pMpu->getFIFOCount();

            do {
	            pMpu->getFIFOBytes(fifoBuffer, packetSize);
                fifoCount-=packetSize;
            } while ( fifoCount < packetSize);

	 		pMpu->dmpGetQuaternion(&q, fifoBuffer);
			pMpu->dmpGetGravity(&gravity, &q);
			pMpu->dmpGetYawPitchRoll(ypr, &q, &gravity);

            //pMpu->dmpGetGyro(gyro_data, fifoBuffer);

            pMpu->getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
            return true;
    }
    else return false;
*/    

}        

void PosEstimator::positionUpdateCallback(double measure_time, double measure_delta_time) {

            
        double wav_l = pLeftWheelRotationSensor->getAngularVelocityAtTime(measure_time);
        double wav_r = pRightWheelRotationSensor->getAngularVelocityAtTime(measure_time);


        double dt_s = measure_time - pos_update_time;

        if (dt_s > measure_delta_time) dt_s = measure_delta_time;

        if (abs(dt_s) > 0.1) return;

        if (updatePosition(wav_l, wav_r, dt_s)) {
            pos_update_time = measure_time;
        }

        pPosController->setCurrentPosition(pos, twist);         
}

bool PosEstimator::updatePosition(double wav_l, double wav_r, double dt_s) {
    if (dt_s == 0) {
        twist.v = 0;
        twist.w = 0;
        return false;        
    }

    updateMpuAngles();

   // printf("w_l: %f  w_r: %f\n", wav_l, wav_r);
	
 	float d_left_m = (wav_l * dt_s * wheel_radius_m);
    float d_right_m = (wav_r * dt_s * wheel_radius_m);

    double d_center_m = (d_right_m + d_left_m) / 2.0;


    double phi_rad = (d_right_m - d_left_m) / wheelbase_m;

    
    float velocity = d_center_m / dt_s;
    float angular_velocity = phi_rad / dt_s;

    //if (twist.v > 0.68 || angular_velocity > 10 ) return false;
    
    if (abs(velocity) > 1 ) {
        twist.v = 0;
        twist.w = 0;
        return false;
    }


    double x_m_dt = d_center_m * std::cos(pos.theta);
    double y_m_dt = d_center_m * std::sin(pos.theta);
    double theta_rad_dt = phi_rad;
    pos.x += x_m_dt;
    pos.y += y_m_dt;


    //pos.x = kalmanX.getAngle(pos.x, x_m_dt/dt_s, dt_s);
    //pos.y = kalmanX.getAngle(pos.y, y_m_dt/dt_s, dt_s);


	w_theta += theta_rad_dt;
    wrapAngle(w_theta);

    //pos.theta = 0.93 * w_theta + 0.07 *( -ypr[0] );

    float mpu_theta = -ypr[0];
    //printf("mpu_theta : %f\n", mpu_theta);
    wrapAngle(mpu_theta);

    float mpu_theta_, pos_theta_;
    if (mpu_theta < 0) mpu_theta_ = mpu_theta + 2*PI;
    else mpu_theta_ = mpu_theta;

    if (pos.theta < 0) pos_theta_ = pos.theta + 2*PI;
    else pos_theta_ = pos.theta;

    float mpu_angular_velocity = gz / 16.4 * PI / 180.0;//abs(mpu_theta_ - pos_theta_) / dt_s;

    float delta_angular_velocity = abs(mpu_angular_velocity - angular_velocity);

    twist.v = velocity;
    twist.w = mpu_angular_velocity;//mpu_angular_velocity; //gx;   //gyro_data[2] / 2000.0 * PI / 180.0;

    pos.theta = mpu_theta;

    return true;

    if (abs(mpu_angular_velocity) < 10)// && delta_angular_velocity < 1)
        pos.theta = mpu_theta;
    else 
        return false;
    //pos.theta = w_theta;

    //pos.theta = kalmanTheta.getAngle(ypr[0], gyro_data[0] / 2000.0 * PI / 180.0,  dt_s);

/*
    if (!wrapAngle(pos.theta))
        pos.theta = kalmanTheta.getAngle(pos.theta, angular_velocity,  dt_s);
    else 
        kalmanTheta.setAngle(pos.theta);
*/

   // w_theta = 0.07*pos.theta + 0.93 * w_theta;

    return true;
}

