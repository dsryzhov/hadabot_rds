#include "MotionController.h"
#include <cstddef>

struct PwValues {
    float pw_l;
    float pw_r;
};

static void motion_control_task(void* arg)
{

	MotionController* controller = static_cast<MotionController*>(arg);

	xQueueHandle motion_control_evt_queue = controller->getEvtQueue();	
    if (motion_control_evt_queue == NULL) return;

    PwValues pv;
    //Motor* pLeftMotor = controller->getLeftMotor();
    //Motor* pRightMotor = controller->getRightMotor();

    

    for(;;) {

        controller->adjustMotorsRotation();    
        vTaskDelay(20/portTICK_PERIOD_MS);
/*
        if(xQueueReceive(motion_control_evt_queue, &pv, 20/portTICK_PERIOD_MS) == pdTRUE ) {
            //pLeftMotor->updateRotation(pv.pw_l);
            //pRightMotor->updateRotation(pv.pw_r);

            controller->updateMotorsRotation();

            //vTaskDelay(20/portTICK_PERIOD_MS);

        } else {
             controller->adjustMotorsRotation();    
        }
*/        
    }
}

MotionController::MotionController()
{
    pLeftMotor = NULL;
    pRightMotor = NULL;
    controlTask = NULL;
    v = w = 0;
    v_l = v_r = 0;

    Kp = 0.1; Ki = 0.1;
    max_wheel_angular_velocity = 0;
    motion_control_evt_queue = xQueueCreate(1, sizeof(PwValues));
    motion_state = NO_MOTION;
}

MotionController::~MotionController() {
    if (controlTask != 0) vTaskDelete(controlTask);

}

void MotionController::init(Motor* _pLeftMotor, Motor* _pRightMotor, RotationSensor* _pLeftRotSensor, RotationSensor* _pRightRotSensor)
{
    pLeftMotor = _pLeftMotor;
    pRightMotor = _pRightMotor;
    pLeftRotSensor = _pLeftRotSensor;
    pRightRotSensor = _pRightRotSensor;
}

void MotionController::setParams(float _wheel_radius_m, float _wheelbase_m, float _max_wheel_angular_velocity) {
    max_wheel_angular_velocity = _max_wheel_angular_velocity; 
    wheel_radius_m = _wheel_radius_m;
    wheelbase_m = _wheelbase_m;
}

void MotionController::begin() {
   xTaskCreatePinnedToCore(motion_control_task, "motion_control", 6144, this, 6, &controlTask, 0);
}

void MotionController::adjustMotorsRotation() {

    if (motion_state == NO_MOTION) return;

    w_l = pLeftRotSensor->getAngularVelocity();
    error_l = v_l - w_l;
    pPart_l = Kp * error_l;
    iPart_l += Ki * error_l;
    //if (iPart_l < 0.0) iPart_l = 0.0;
    w_l_f = pPart_l + iPart_l;
    pw_l_f = w_l_f / max_wheel_angular_velocity;
    

    w_r = pRightRotSensor->getAngularVelocity();
    error_r = v_r - w_r;
    pPart_r = Kp * error_r;
    iPart_r += Ki * error_r;
    //if (iPart_r < 0.0) iPart_r = 0.0;
    w_r_f = pPart_r + iPart_r;
    pw_r_f = w_r_f / max_wheel_angular_velocity;


    if  (pw_r_f > 1.0) pw_r_f = 1.0;
    else if (pw_r_f < -1.0) pw_r_f = -1.0;
    if  (pw_l_f > 1.0) pw_l_f = 1.0;
    else if (pw_l_f < -1.0) pw_l_f = -1.0;

    //printf("pw_l_f : %f pw_r_f : %f", pw_l_f, pw_r_f);
    pLeftMotor->updateRotation(pw_l_f);
    pRightMotor->updateRotation(pw_r_f);      

    if (pw_l_f ==0 && pw_r_f == 0) {
        motion_state = NO_MOTION;
    }
}

void MotionController::updateMotorsRotation() {
    pLeftMotor->updateRotation(pw_l);
    pRightMotor->updateRotation(pw_r);
}

void MotionController::updateMotion(float _linear_velocity, float _angular_velocity)
{
    v = _linear_velocity;
    w = _angular_velocity;
    
    v_r = ((2.0 * v) + (w * wheelbase_m)) / (2 * wheel_radius_m);
    v_l = ((2.0 * v) - (w * wheelbase_m)) / (2 * wheel_radius_m);
    
    pw_r = v_r / max_wheel_angular_velocity; // 21 - max possible rad/s for my motors
    pw_l = v_l / max_wheel_angular_velocity;


    if  (pw_r > 1.0) pw_r = 1.0;
    else if (pw_r < -1.0) pw_r = -1.0;
    if  (pw_l > 1.0) pw_l = 1.0;
    else if (pw_l < -1.0) pw_l = -1.0;

    pPart_l = iPart_l = 0;
    pPart_r = iPart_r = 0;

    if (pLeftMotor != NULL && pRightMotor != NULL) {
        pLeftMotor->updateRotation(pw_l);
        pRightMotor->updateRotation(pw_r);   
        if (pw_l != 0 || pw_r != 0) 
            motion_state = IN_MOTION;     
        else 
            motion_state = NO_MOTION;
    }
/*
    if (pLeftMotor != NULL && pRightMotor != NULL) {
        
        PwValues pw_values;
        pw_values.pw_l = pw_l;
        pw_values.pw_r = pw_r;
	    xQueueSendFromISR(motion_control_evt_queue, &pw_values, NULL);	        
        
        //pLeftMotor->updateRotation(pw_l);
        //pRightMotor->updateRotation(pw_r);
    }
*/
}