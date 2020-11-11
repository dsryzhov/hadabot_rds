#ifndef __IMOTION_CONTROLLER_H
#define __IMOTION_CONTROLLER_H

class IMotionController {
    public:
        virtual void updateMotion(float linear_velocity, float angular_velocity) = 0;

};

#endif