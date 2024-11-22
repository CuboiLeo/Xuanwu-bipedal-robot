#ifndef ROBOT_TYPES_H
#define ROBOT_TYPES_H

struct Joint_Angle { // Joint angles in radians
    float hip_yaw;
    float hip_roll;
    float hip_pitch;
    float knee_pitch;
};

struct Direction_Vector { // Direction_Vector in m, m/s, m/s^2
    float x;
    float y;
    float z;
};

struct Direction_Vector_Two { // Direction_Vector_Two in m, m/s, m/s^2
    Direction_Vector left;
    Direction_Vector right;
};

struct DH_Parameter {
        float a;      // Link length
        float alpha;  // Link twist
        float d;      // Link offset
        float theta;  // Joint angle
};

#endif // ROBOT_TYPES_H