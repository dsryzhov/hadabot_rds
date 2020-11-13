#ifndef __POS_CONTROLLER_
#define __POS_CONTROLLER_

#include "IPosController.h"
#include "MotionController.h"

enum MotionType {
    STOP, ROTATE_CW, ROTATE_CCW, MOVE_FORWARD, MOVE_BACKWARD
};

enum PosControllerMode {
    IDLE, NAVIGATION
};

enum PosState {
    POS_START_ROTATION, POS_ROTATION, POS_STOPED, POS_START_MOVE, POS_MOVE
};

class PosController : public IPosController {
public:
    PosController();
    ~PosController();

    void init(IMotionController* _pMotionController);

    void setCurrentPosition(Position _current_position, Twist _twist) override;
    void setGoalPosition(Position _goal_position) override;

    void stopNavigation(const char* from);

    void setParams(float _desired_linear_velocity,
              float _desired_angular_velocity, 
              float _distance2goal_thresh, 
              float _angle2goal_pos_thresh, 
              float _angle2goal_orientation_thresh,
              float _pos_control_period_ms);
    
    inline IMotionController* getMotionController() { return pMotionController;}

    inline MotionType getMotionType() {return current_motion_type; }

    inline TaskHandle_t getEvtQueue() { return pos_control_evt_queue; }

    
    inline float calcDistanceToGoal();
    inline float calcAngle2GoalPosition();
    inline float calcAngle2GoalOrientation();
    void calcLocalGoalPose(float &gx, float &gy);

    void controlDelay();
    void updateMotion();
    virtual bool updateMotionType(MotionType new_motion_type);

    virtual MotionType selectNeededMotionType() = 0;
    virtual void MoveStep(int dir ) = 0;
    virtual void RotateStep(int dir);
        

protected:
    PosControllerMode controller_mode;
    PosState position_state;

    TaskHandle_t posControlTask;
    xQueueHandle pos_control_evt_queue = NULL;

    IMotionController* pMotionController;

    Position current_position; // in odometry frame
    Twist current_twist;
    MotionType current_motion_type;

    Position goal_position;  // in odometry frame

    float distance2goal;
    float angle2goal_pos;
    float angle2goal_orientation;
    float linear_velocity;
    float angular_velocity;
    float angle_error;
    float pos_error;

    float distance2goal_thresh;
    float angle2goal_pos_thresh;
    float angle2goal_orientation_thresh;
    float desired_linear_velocity;
    float desired_angular_velocity;
    float pos_control_period_ms;

    float iPart;
    float Ki;
    float Kp;
};

#endif
