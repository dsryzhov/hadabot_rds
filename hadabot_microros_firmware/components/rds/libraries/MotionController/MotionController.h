#ifndef __MOTION_CONTROLLER_H
#define __MOTION_CONTROLLER_H

#include "IMotionController.h"
#include "motor.h"
#include "rotsensor.h"
#include "Arduino.h"

enum MotionState {
    NO_MOTION, IN_MOTION
};

class MotionController : public IMotionController {
public:
    MotionController();
    ~MotionController();

    void init(Motor* _pLeftMotor, Motor* _pRightMotor, RotationSensor* _pLeftRotSensor, RotationSensor* _pRightRotSensor);
    void setParams(float _wheel_radius_m, float _wheelbase_m, float _max_wheel_angular_velocity);
    void begin();

    void updateMotion(float linear_velocity, float angular_velocity) override;


    inline Motor* getLeftMotor() { return pLeftMotor; }
    inline Motor* getRightMotor() { return pRightMotor; }

    void adjustMotorsRotation();
    void updateMotorsRotation();

    inline xQueueHandle getEvtQueue() { return motion_control_evt_queue;}

protected:
    volatile MotionState motion_state;

    float v;
    float w;
    float max_wheel_angular_velocity;
    float v_r;
    float v_l;
    float pw_r;
    float pw_l;

    float Kp, Ki;

    float w_l;
    float error_l;
    float pPart_l;
    float iPart_l;
    float w_l_f;
    float pw_l_f;

    float w_r;
    float error_r;
    float pPart_r;
    float iPart_r;
    float w_r_f;
    float pw_r_f;    

    float wheelbase_m;
    float wheel_radius_m;

    Motor* pLeftMotor;
    Motor* pRightMotor;

    RotationSensor* pLeftRotSensor;
    RotationSensor* pRightRotSensor;

    xQueueHandle motion_control_evt_queue = NULL;

    TaskHandle_t controlTask;

};

#endif