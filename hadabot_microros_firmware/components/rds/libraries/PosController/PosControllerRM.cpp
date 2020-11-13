#include "PosControllerRM.h"



PosControllerRM::PosControllerRM(): PosController() 
{
}

PosControllerRM::~PosControllerRM() {

}

MotionType PosController::selectNeededMotionType() {

    MotionType needed_motion_type;

    distance2goal = calcDistanceToGoal();
    if (abs(distance2goal) > distance2goal_thresh) {
        // move to goal position
        angle2goal_pos = calcAngle2GoalPosition();
        if (abs(angle2goal_pos) > angle2goal_pos_thresh) {
            // rotate to goal position direction 
            if  (angle2goal_pos >= 0) needed_motion_type = ROTATE_CCW;
            else needed_motion_type = ROTATE_CW;
            angle_error = angle2goal_pos;
        } else {
            // move to goal position
            if (distance2goal >= 0)
                needed_motion_type = MOVE_FORWARD;
            else 
                needed_motion_type = MOVE_BACKWARD;
            pos_error = distance2goal;
        }

    } else {
    // rotate to goal orientation
        angle2goal_orientation = calcAngle2GoalOrientation();
        if (abs(angle2goal_orientation) > angle2goal_orientation_thresh) {
            // rotate to goal orinentation
            if  (angle2goal_orientation >= 0) needed_motion_type = ROTATE_CCW;
            else needed_motion_type = ROTATE_CW;
            angle_error = angle2goal_orientation;
        } else {
            // stop 
            needed_motion_type = STOP;
        }

    }
    printf("Motion type: %d\n", (int)needed_motion_type);
    return needed_motion_type;
}

void PosController::MoveStep(int dir) {

    Ki = 1;
    Kp = 2.25;
    float min_linear_velocity = 0.3;

    if (position_state == POS_START_MOVE) {
        iPart += desired_linear_velocity*Ki*dir;
        linear_velocity = iPart;
    }
    else {
        linear_velocity = min_linear_velocity*dir + pos_error*Kp;
    }

    if (abs(linear_velocity) >= desired_linear_velocity) {
        linear_velocity = desired_linear_velocity*dir;
        position_state = POS_MOVE;
    }
    pMotionController->updateMotion(linear_velocity, 0);
    printf("Linear_velocity: %f pos_error: %f \n", linear_velocity, pos_error );
}
