#ifndef ROBOT_TYPES_H
#define ROBOT_TYPES_H

#include <stdint.h>
#include <iostream>

struct Joint_Angles { // Joint angles in radians
    float hip_yaw;
    float hip_roll;
    float hip_pitch;
    float knee_pitch;
};

struct Direction_Vector { // Direction vector in m, m/s, m/s^2
    float x;
    float y;
    float z;
};

struct Joint_Data {
    Joint_Angles ref;
    Joint_Angles act;
};

struct Leg_Data {
    Joint_Data left;
    Joint_Data right;
};

struct Direction_Data {
    Direction_Vector ref;
    Direction_Vector act;
};

struct Foot_Data {
    Direction_Data left;
    Direction_Data right;
};

#endif // ROBOT_TYPES_H