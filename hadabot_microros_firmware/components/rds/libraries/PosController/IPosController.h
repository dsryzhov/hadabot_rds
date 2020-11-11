#ifndef __IPOS_CONTROLLER_
#define __IPOS_CONTROLLER_

#include "Geometry.h"

class IPosController {
    public:
        virtual void setCurrentPosition(Position _current_position, Twist _twist) = 0;
        virtual void setGoalPosition(Position _goal_position) = 0;


};

#endif