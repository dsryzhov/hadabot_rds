#include "PosPurePursuitController.h"



PosPurePursuitController::PosPurePursuitController() : PosController()
{
    max_angular_vel_fw_motion = 1.2; // rad/s
}

PosPurePursuitController::~PosPurePursuitController() {

}

float PosPurePursuitController::calcDistanceToGoal() {
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


float PosPurePursuitController::calcAngle2GoalPosition() {

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

void PosPurePursuitController::calcLocalGoalPose(float &gx, float &gy)
 {
    float th = current_position.theta;
    float cos_th = cos(th);
    float sin_th = sin(th);

    float dx = goal_position.x - current_position.x;
    float dy = goal_position.y - current_position.y;

    gx = dx*cos_th + dy*sin_th;
    gy = -dx*sin_th + dy*cos_th;
     
 }


MotionType PosPurePursuitController::selectNeededMotionType() {

    MotionType needed_motion_type;

    distance2goal = PosPurePursuitController::calcDistanceToGoal();
    if (abs(distance2goal) > distance2goal_thresh) {
        // move to goal position
        angle2goal_pos = PosPurePursuitController::calcAngle2GoalPosition();
        if (abs(angle2goal_pos) >= 0.5*PI) {
            // rotate to goal position direction 
            if  (angle2goal_pos >= 0) needed_motion_type = ROTATE_CCW;
            else needed_motion_type = ROTATE_CW;
            angle_error = angle2goal_pos;
        } else {
            needed_motion_type = MOVE_FORWARD;
            pos_error = distance2goal;
        }
    }  else 
        needed_motion_type = STOP;

    printf("Motion type: %d\n", (int)needed_motion_type);
    return needed_motion_type;
}

void PosPurePursuitController::MoveStep(int dir) {

    float xl, yl;

    PosPurePursuitController::calcLocalGoalPose(xl, yl);

    float curvature = 2.0 * yl / (xl * xl + yl * yl);

    linear_velocity = desired_linear_velocity;
    angular_velocity = desired_linear_velocity * curvature;

    if (angular_velocity < -max_angular_vel_fw_motion) angular_velocity = -max_angular_vel_fw_motion;
    if (angular_velocity > max_angular_vel_fw_motion) angular_velocity = max_angular_vel_fw_motion;

    printf("Linear_velocity: %f angular_velocity: %f \n", linear_velocity, angular_velocity );

    pMotionController->updateMotion(linear_velocity, angular_velocity);
    //printf("Linear_velocity: %f pos_error: %f \n", linear_velocity, pos_error );
}





