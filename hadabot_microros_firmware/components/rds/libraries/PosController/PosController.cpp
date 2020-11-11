#include "PosController.h"


static void position_control_task(void* arg)
{

	PosController* controller = static_cast<PosController*>(arg);

	//xQueueHandle pos_control_evt_queue = pos_controller->getEvtQueue();	
    //if (pos_control_evt_queue == NULL) return;

    while(1) {
        controller->updateMotion();   
        controller->controlDelay();
    }
}

PosController::PosController() : current_position(0,0,0)
{
    controller_mode = IDLE;
    pMotionController = NULL;
    posControlTask = NULL;
    current_motion_type = STOP;
    distance2goal_thresh = 0.03; // meter
    angle2goal_pos_thresh = 0.3; // rad
    angle2goal_orientation_thresh = 0.3; // rad
    desired_linear_velocity = 0.45; // meters
    desired_angular_velocity = 7.85; // radians
    pos_control_period_ms = 20;

}

PosController::~PosController() {
    stopNavigation();
}

void PosController::setParams(float _desired_linear_velocity,
              float _desired_angular_velocity,
              float _distance2goal_thresh, 
              float _angle2goal_pos_thresh, 
              float _angle2goal_orientation_thresh,
              float _pos_control_period_ms) {
    desired_linear_velocity = _desired_linear_velocity;
    desired_angular_velocity = _desired_angular_velocity;
    distance2goal_thresh = _distance2goal_thresh;
    angle2goal_pos_thresh = _angle2goal_pos_thresh;
    angle2goal_orientation_thresh = _angle2goal_orientation_thresh;
    pos_control_period_ms = _pos_control_period_ms;
}

void PosController::init(IMotionController* _pMotionController) {
    pMotionController = _pMotionController;
    //pos_control_evt_queue = xQueueCreate(1, sizeof());
}


void PosController::setCurrentPosition(Position _current_position, Twist _twist) {
    current_position = _current_position;
    current_twist = _twist;
    //printf("current position x: %f y: %f theta %f\n", current_position.x, current_position.y, current_position.theta);
}

void PosController::setGoalPosition(Position _goal_position) {
    float th = current_position.theta;
    float cos_th = cos(th);
    float sin_th = sin(th);
    float dx = _goal_position.x;
    float dy = _goal_position.y;

    goal_position.x = current_position.x + dx*cos_th - dy*sin_th;
    goal_position.y = current_position.y + dx*sin_th + dy*cos_th;
    goal_position.theta = current_position.theta + _goal_position.theta;
    wrapAngle(goal_position.theta);

    printf("New goal position xw : %f yw: %f theta: %f\n", goal_position.x, goal_position.y, goal_position.theta);    

    controller_mode = NAVIGATION;
    if (posControlTask == NULL)
        xTaskCreatePinnedToCore(position_control_task, "pos_control", 6144, this, 6, &posControlTask, 0);
}

void PosController::controlDelay() {
    vTaskDelay(pos_control_period_ms/portTICK_PERIOD_MS);
}

void PosController::stopNavigation() {
    controller_mode = IDLE;
   
/*   
    if (posControlTask != NULL) {
        vTaskDelete(posControlTask);
        posControlTask = NULL;
    }
  */  
}

void PosController::updateMotion() {
    if (controller_mode == NAVIGATION) {
        MotionType new_motion_type = selectNeededMotionType();
        updateMotionType(new_motion_type);
        if (new_motion_type == STOP)  stopNavigation();
    }
}


inline float PosController::calcDistanceToGoal() {
    float dx = goal_position.x - current_position.x;
    float dy = goal_position.y - current_position.y;
    float distance = sqrt(dx*dx+dy*dy);

    printf("Distance to goal: %f\n", distance);

    return distance;
}

inline float PosController::calcAngle2GoalPosition() {
    float dx = goal_position.x - current_position.x;
    float dy = goal_position.y - current_position.y;

    float pos_angle = atan2(dy, dx);
  
    float delta_angle = pos_angle - current_position.theta;

    //printf("dx: %f dy: %f pos_angle: %f delta_angle: %f\n", dx, dy, pos_angle, delta_angle);

    wrapAngle(delta_angle);

    printf("Angle to goal position: %f\n", delta_angle);

    return delta_angle;
}


inline float PosController::calcAngle2GoalOrientation() {
    float delta_theta = goal_position.theta - current_position.theta;
    wrapAngle(delta_theta);
    printf("Angle to goal orientation: %f\n", delta_theta);    
    return delta_theta;
}

MotionType PosController::selectNeededMotionType() {

    MotionType needed_motion_type;

    distance2goal = calcDistanceToGoal();
    if (distance2goal > distance2goal_thresh) {
        // move to goal position
        angle2goal_pos = calcAngle2GoalPosition();
        if (abs(angle2goal_pos) > angle2goal_pos_thresh) {
            // rotate to goal position direction 
            if  (angle2goal_pos >= 0) needed_motion_type = ROTATE_CCW;
            else needed_motion_type = ROTATE_CW;
        } else {
            // move to goal position
            needed_motion_type = MOVE_FORWARD;
        }

    } else {
    // rotate to goal orientation
        angle2goal_orientation = calcAngle2GoalOrientation();
        if (abs(angle2goal_orientation) > angle2goal_orientation_thresh) {
            // rotate to goal orinentation
            if  (angle2goal_orientation >= 0) needed_motion_type = ROTATE_CCW;
            else needed_motion_type = ROTATE_CW;
        } else {
            // stop 
            needed_motion_type = STOP;
        }

    }
    printf("Motion type: %d\n", (int)needed_motion_type);
    return needed_motion_type;
}

bool PosController::updateMotionType(MotionType new_motion_type) {
    bool flMotionUpdated;

    if (current_motion_type != new_motion_type) {
        linear_velocity = 0;
        angular_velocity = 0;
        switch (new_motion_type)  {
            case MOVE_FORWARD: linear_velocity = desired_linear_velocity; break;
            case MOVE_BACKWARD: linear_velocity = -desired_linear_velocity; break;
            case ROTATE_CW: angular_velocity = -desired_angular_velocity; break;
            case ROTATE_CCW: angular_velocity = desired_angular_velocity; break;
            default: break;
        }
        printf("Set motion wint linear velocity: %f and angular velocity: %f\n", linear_velocity, angular_velocity);
        pMotionController->updateMotion(linear_velocity, angular_velocity);
        current_motion_type = new_motion_type;
        flMotionUpdated = true;            
    } else flMotionUpdated = false;
    return flMotionUpdated;
}

