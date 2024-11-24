#ifndef ROBOT_TYPES_H
#define ROBOT_TYPES_H

struct joint_angles { // Joint angles in radians
    float hip_yaw;
    float hip_roll;
    float hip_pitch;
    float knee_pitch;
};

struct direction_vector { // Direction_Vector in m, m/s, m/s^2
    float x;
    float y;
    float z;
};

struct DH_parameter {
        float a;      // Link length
        float alpha;  // Link twist
        float d;      // Link offset
        float theta;  // Joint angle
};

struct joint_data {
    joint_angles ref;
    joint_angles act;
};

struct leg_data {
    joint_data left;
    joint_data right;
};

struct direction_data {
    direction_vector ref;
    direction_vector act;
};

struct foot_data {
    direction_data left;
    direction_data right;
};

struct CoM_data {
    direction_vector ref;
    direction_vector act;
};

struct ZMP_data {
    direction_vector ref;
    direction_vector act;
};


#endif // ROBOT_TYPES_H