#include "Geometry.h"

bool wrapAngle(float &angle) {
    bool flRes = false;
    while (angle > PI || angle < -PI) {
        if (angle > PI) {
            angle = angle - 2*PI ;
            flRes = true;
        }
        else 
            if (angle < -PI) {
                angle = angle + 2*PI;
                flRes = true;
            }
    }
    return flRes;
}
