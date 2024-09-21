#ifndef ROBOT_TYPES_H
#define ROBOT_TYPES_H

#ifdef __cplusplus
extern "C" {
#endif

struct Joint_Angle { // Joint angles in radians
    float hip_yaw;
    float hip_roll;
    float hip_pitch;
    float knee_pitch;
};

struct Foot_Position { // Foot position wrt Central Connector Module in meters
    float x;
    float y;
    float z;
};

struct DH_Parameter {
        float a;      // Link length
        float alpha;  // Link twist
        float d;      // Link offset
        float theta;  // Joint angle
};

enum motor_name {
    Left_Hip_Yaw,
    Left_Hip_Roll,
    Left_Hip_Pitch,
    Left_Knee_Pitch,
    Right_Hip_Yaw,
    Right_Hip_Roll,
    Right_Hip_Pitch,
    Right_Knee_Pitch
};

#ifdef __cplusplus
}
#endif

#endif // ROBOT_TYPES_H