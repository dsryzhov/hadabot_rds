#ifndef __GEOMETRY_H
#define __GEOMETRY_H

struct Position {
	float x, y, theta;
    Position() {}
    Position(float _x, float _y, float _theta) {x = _x; y = _y; theta = _theta;};
};

struct Twist {
    float v;
    float w;
};

#define PI 3.141596

bool wrapAngle(float &angle);

#endif