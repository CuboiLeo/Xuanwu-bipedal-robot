#ifndef ROBOT_H
#define ROBOT_H

#ifdef __cplusplus
extern "C" {
#endif
 
#include "stdint.h"
#include "User_Math.h"

struct Joint_Angle { // Joint angles in radians
    float hip_yaw;
    float hip_roll;
    float hip_pitch;
    float knee_pitch;
};

struct Foot_Position { // Foot position w.r.t Central Connector Module in meters
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

class Robot {
public:
    static constexpr int NUM_JOINTS = 5;
    static constexpr DH_Parameter DH_Left_Leg[NUM_JOINTS] = {
        // a      alpha   d       theta
        {-0.135f, 0,      0,      0}, // Base - Hip Yaw
        {-0.095f, -PI/2,  0,      0}, // Hip Yaw - Hip Roll
        {-0.09f,  -PI/2,  0,      0}, // Hip Roll - Hip Pitch
        {-0.18f,  0,      0,      0}, // Hip Pitch - Knee Pitch
        {-0.38f,  0,      0,      0}  // Knee Pitch - Foot
    };
    static constexpr DH_Parameter DH_Right_Leg[NUM_JOINTS] = {
        // a      alpha   d       theta
        {0.135f, 0,       0,      0}, // Base - Hip Yaw
        {0.095f,  PI/2,   0,      0}, // Hip Yaw - Hip Roll
        {-0.09f, -PI/2,   0,      0}, // Hip Roll - Hip Pitch
        {-0.18f,  0,      0,      0}, // Hip Pitch - Knee Pitch
        {-0.38f,  0,      0,      0}  // Knee Pitch - Foot
    };

    Robot();
    // Public interface for interacting with joint angles
    Joint_Angle getActJointAnglesLeft() const;
    Joint_Angle getActJointAnglesRight() const;
    void setActJointAnglesLeft(const Joint_Angle& angles);
    void setActJointAnglesRight(const Joint_Angle& angles);
    Joint_Angle getRefJointAnglesLeft() const;
    Joint_Angle getRefJointAnglesRight() const;
    void setRefJointAnglesLeft(const Joint_Angle& angles);
    void setRefJointAnglesRight(const Joint_Angle& angles);

    // Public interface for interacting with foot position
    Foot_Position getRefFootPosLeft() const;
    Foot_Position getRefFootPosRight() const;
    void setRefFootPosLeft(const Foot_Position& position);
    void setRefFootPosRight(const Foot_Position& position);
    Foot_Position getActFootPosLeft() const;
    Foot_Position getActFootPosRight() const;
    void setActFootPosLeft(const Foot_Position& position);
    void setActFootPosRight(const Foot_Position& position);

    // Public interface for soft start flag
    uint8_t getSoftStartFlag() const;
    void setSoftStartFlag(uint8_t flag);

private:
    // Internal state of the robot
    Joint_Angle act_left_leg_angles;    // Actual left leg joint angles
    Joint_Angle act_right_leg_angles;   // Actual right leg joint angles
    Joint_Angle ref_left_leg_angles;    // Reference left leg joint angles through inverse kinematics
    Joint_Angle ref_right_leg_angles;   // Reference right leg joint angles through inverse kinematics

    Foot_Position ref_left_foot_pos;  // Reference left foot position
    Foot_Position ref_right_foot_pos; // Reference right foot position
    Foot_Position act_left_foot_pos;  // Actual left foot position through forward kinematics
    Foot_Position act_right_foot_pos; // Actual right foot position through forward kinematics

    uint8_t soft_start_flag;
};

extern Robot robot;
extern void Robot_Task(void const* argument);

#ifdef __cplusplus
}
#endif

#endif
