#include "PosEstimator.h"

PosEstimator::PosEstimator(float _wheel_radius_m, float _wheelbase_m) 
: wheel_radius_m{_wheel_radius_m}, wheelbase_m{_wheelbase_m}
{
    pos_update_time = 0;
	pos.x = pos.y = pos.theta = 0;    
}

void PosEstimator::init(RotationSensor* _pLeftWheelRotationSensor, RotationSensor* _pRightWheelRotationSensor) {
    pLeftWheelRotationSensor = _pLeftWheelRotationSensor;
    pRightWheelRotationSensor = _pRightWheelRotationSensor;
}

void PosEstimator::positionUpdateCallback(double measure_time, double measure_delta_time) {
		
    double wav_l = pLeftWheelRotationSensor->getAngularVelocityAtTime(measure_time);
    double wav_r = pRightWheelRotationSensor->getAngularVelocityAtTime(measure_time);


    double dt_s = measure_time - pos_update_time;

    if (dt_s > measure_delta_time) dt_s = measure_delta_time;

    updatePosition(wav_l, wav_r, dt_s);

	pos_update_time = measure_time;

}

void PosEstimator::updatePosition(double wav_l, double wav_r, double dt_s) {
	
 	float d_left_m = (wav_l * dt_s * wheel_radius_m);
    float d_right_m = (wav_r * dt_s * wheel_radius_m);

    double d_center_m = (d_right_m + d_left_m) / 2.0;
    double phi_rad = (d_right_m - d_left_m) / wheelbase_m;

    double x_m_dt = d_center_m * std::cos(pos.theta);
    double y_m_dt = d_center_m * std::sin(pos.theta);
    double theta_rad_dt = phi_rad;

    pos.x += x_m_dt;
    pos.y += y_m_dt;
	pos.theta += theta_rad_dt;
}