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
    stopNavigation("Destructor");
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
    current_position.x = _current_position.x;
    current_position.y = _current_position.y;
    current_position.theta = _current_position.theta;
    current_twist.v = _twist.v;
    current_twist.w = _twist.w;
    printf("current position x: %f y: %f theta %f\n", current_position.x, current_position.y, current_position.theta);
}

void PosController::setGoalPosition(Position local_goal_position) {
    if (local_goal_position.x == 0 && local_goal_position.y == 0 && local_goal_position.theta == 0) {
        if (controller_mode == NAVIGATION) stopNavigation("setGoalPosition");
        return;
    }

    float th = current_position.theta;
    float cos_th = cos(th);
    float sin_th = sin(th);
    float dx = local_goal_position.x;
    float dy = local_goal_position.y;

    goal_position.x = current_position.x + dx*cos_th - dy*sin_th;
    goal_position.y = current_position.y + dx*sin_th + dy*cos_th;

    float alfa = atan2(dy, dx);

    goal_position.theta = current_position.theta + alfa + local_goal_position.theta;
    wrapAngle(goal_position.theta);

    printf("New goal position xw : %f yw: %f theta: %f\n", goal_position.x, goal_position.y, goal_position.theta);    

    controller_mode = NAVIGATION;
    if (posControlTask == NULL)
        xTaskCreatePinnedToCore(position_control_task, "pos_control", 6144, this, 6, &posControlTask, 0);
}

void PosController::controlDelay() {
    vTaskDelay(pos_control_period_ms/portTICK_PERIOD_MS);
}

void PosController::stopNavigation(const char* from) {
    controller_mode = IDLE;
    pMotionController->updateMotion(0, 0);
    iPart = 0;
    current_motion_type = STOP;
    position_state = POS_STOPED;
    angle_error = 0;
    pos_error = 0;
    printf("Navigation is stoped from %s\n", from);
/*   
    if (posControlTask != NULL) {
        vTaskDelete(posControlTask);
        posControlTask = NULL;
    }
  */  
}



float PosController::calcDistanceToGoal() {
    float dx = goal_position.x - current_position.x;
    float dy = goal_position.y - current_position.y;
    float distance = sqrt(dx*dx+dy*dy);

    float distanse_cos_alfa = dx*cos(current_position.theta) + dy*sin(current_position.theta);
    printf("distanse_cos_alfa %f\n", distanse_cos_alfa);

    if (distanse_cos_alfa < 0) distance*=-1; 

    //float distance = dx;

    printf("Distance to goal: %f\n", distance);

    return distance;
}


float PosController::calcAngle2GoalPosition() {

    float dx = goal_position.x - current_position.x;
    float dy = goal_position.y - current_position.y;

    float pos_theta = atan2(dy, dx);

    float cur_theta = current_position.theta;

    if (pos_theta < 0) pos_theta+= 2*PI;
    if (cur_theta < 0) cur_theta+= 2*PI;

  
    float delta_theta = pos_theta - cur_theta;

    //printf("dx: %f dy: %f pos_angle: %f delta_angle: %f\n", dx, dy, pos_angle, delta_angle);

    wrapAngle(delta_theta);

    printf("Angle to goal position: %f\n", delta_theta);

    return delta_theta;
}

void PosController::calcLocalGoalPose(float &gx, float &gy)
 {
    float th = current_position.theta;
    float cos_th = cos(th);
    float sin_th = sin(th);

    float dx = goal_position.x - current_position.x;
    float dy = goal_position.y - current_position.y;

    gx = dx*cos_th + dy*sin_th;
    gy = -dx*sin_th + dy*cos_th;
     
 }

float PosController::calcAngle2GoalOrientation() {

    float goal_theta = goal_position.theta;
    float cur_theta = current_position.theta;

    if (goal_theta < 0) goal_theta+= 2*PI;
    if (cur_theta < 0) cur_theta+= 2*PI;

    float delta_theta = goal_theta - cur_theta;
    //float delta_theta = goal_position.theta - current_position.theta;
    wrapAngle(delta_theta);
    printf("Angle to goal orientation: %f\n", delta_theta);    
    return delta_theta;
}

void PosController::updateMotion() {
    if (controller_mode == NAVIGATION) {
        MotionType new_motion_type = selectNeededMotionType();
        updateMotionType(new_motion_type);
        //if (position_state == POS_STOPED)  stopNavigation("updateMotion");
    }
}

bool PosController::updateMotionType(MotionType new_motion_type) {
    bool flMotionUpdated;

    if (current_motion_type != new_motion_type) {
        iPart = 0;
        if (new_motion_type == ROTATE_CW || new_motion_type == ROTATE_CCW)
            position_state = POS_START_ROTATION;
        
        if (new_motion_type == MOVE_FORWARD || new_motion_type == MOVE_BACKWARD)
            position_state = POS_START_MOVE;
    } else {

        //if (position_state == POS_ROTATION && current_twist.w == 0)
        //    stopNavigation("updateMotionType 1");

        //if (position_state == POS_MOVE && current_twist.v == 0)
          //  stopNavigation("updateMotionType 2");
    }

    linear_velocity = 0;
    angular_velocity = 0;
    switch (new_motion_type)  {
        case MOVE_FORWARD: MoveStep(1); break;
        case MOVE_BACKWARD: MoveStep(-1); break;
        case ROTATE_CW: RotateStep(-1); break;
        case ROTATE_CCW: RotateStep(1); break;
        case STOP : stopNavigation("updateMotionType-swicth"); 
        default: break;
    }
    //printf("Set motion wint linear velocity: %f and angular velocity: %f\n", linear_velocity, angular_velocity);
    current_motion_type = new_motion_type;

    flMotionUpdated = true;            

    return flMotionUpdated;
}

void PosController::RotateStep(int dir) {

    Ki = 0.1;
    Kp = 9.72;
    float min_angular_velocity = 1; // rad

    if (position_state == POS_START_ROTATION ) {
        iPart += desired_angular_velocity*Ki*dir;

        angular_velocity = iPart;
    } 
    else {
        angular_velocity = min_angular_velocity*dir + angle_error*Kp;  
       // printf("POS_ROTATION\n");      
    }
    
    //angular_velocity = iPart + angle_error*Kp;


    if (abs(angular_velocity) >= desired_angular_velocity) {
        angular_velocity = desired_angular_velocity*dir;
        position_state = POS_ROTATION;
    }
    //printf("Angular_velocity: %f error: %f \n", angular_velocity, angle_error );
    pMotionController->updateMotion(0, angular_velocity);

}


