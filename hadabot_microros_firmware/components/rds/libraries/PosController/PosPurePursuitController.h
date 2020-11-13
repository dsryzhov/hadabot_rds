#ifndef __POS_PURE_PURSUIT_CONTROLLER_
#define __POS_PURE_PURSUIT_CONTROLLER_

#include "PosController.h"

class PosPurePursuitController : public PosController {
public:

    PosPurePursuitController();
    ~PosPurePursuitController();

    float calcDistanceToGoal();
    float calcAngle2GoalPosition();
    void calcLocalGoalPose(float &gx, float &gy);    

    MotionType selectNeededMotionType() override;

    void MoveStep(int dir ) override;
       

protected:
    float max_angular_vel_fw_motion;
};

#endif
