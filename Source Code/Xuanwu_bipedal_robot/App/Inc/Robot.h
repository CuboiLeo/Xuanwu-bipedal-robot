#ifndef ROBOT_H
#define ROBOT_H

#include "stdint.h"
#include "User_Math.h"

#define DH_A1 (-0.135f)
#define DH_A2 (-0.095f)
#define DH_A3 (-0.09f)
#define DH_A4 (-0.18f)
#define DH_A5 (-0.38f)
#define DH_A6 (0.135f)
#define DH_A7 (0.095f)
#define DH_A8 (-0.09f)
#define DH_A9 (-0.18f)
#define DH_A10 (-0.38f)

#define DH_ALPHA2 (-PI/2)
#define DH_ALPHA3 (-PI/2)
#define DH_ALPHA7 (PI/2)
#define DH_ALPHA8 (-PI/2)

typedef struct Joint_Angle
{
    float hip_yaw;
    float hip_roll;
    float hip_pitch;
    float knee_pitch;
}Joint_Angle_t;                     // joint angles in radians

typedef struct Foot_Position
{
    float x;
    float y;
    float z;
}Foot_Position_t;                   // foot position in meters
typedef struct Robot
{
    uint8_t soft_start_flag;        // 1: all joints zeroed, 0: not all joints zeroed
    Joint_Angle_t left_leg;         // left leg joint angles using encoder feedback
    Joint_Angle_t right_leg;        // right leg joint angles using encoder feedback
    Foot_Position_t left_foot;      // left foot position using forward kinematics
    Foot_Position_t right_foot;     // right foot position using forward kinematics
    Joint_Angle_t IK_left_leg;      // left leg joint angles using inverse kinematics
    Joint_Angle_t IK_right_leg;     // right leg joint angles using inverse kinematics
    Foot_Position_t ref_left_foot;  // left foot position reference for inverse kinematics
    Foot_Position_t ref_right_foot; // right foot position reference for inverse kinematics
} Robot_t;

void Robot_Joint_Angle_Assign(void);

extern Robot_t g_Robot;
#endif
