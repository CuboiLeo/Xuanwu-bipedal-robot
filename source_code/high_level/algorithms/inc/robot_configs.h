#ifndef ROBOT_CONFIGS_H
#define ROBOT_CONFIGS_H

#include "robot_types.h"
#include "user_math.h"

class Robot
{
public:
    Robot();
private:
    leg_data leg_angles;
    foot_data foot_pos;
    CoM_data CoM_pos;
    ZMP_data ZMP_pos;
};

// Number of joints per leg, end effector counts
static constexpr int NUM_JOINTS = 5;

// Denavit-Hartenberg parameters for the robot
static constexpr DH_parameter DH_Left_Leg[NUM_JOINTS] = {
    // a      alpha   d       theta
    {-0.135f, 0, 0, 0},       // Base - Hip Yaw
    {-0.095f, -PI / 2, 0, 0}, // Hip Yaw - Hip Roll
    {-0.09f, -PI / 2, 0, 0},  // Hip Roll - Hip Pitch
    {-0.18f, 0, 0, 0},        // Hip Pitch - Knee Pitch
    {-0.38f, 0, 0, 0}         // Knee Pitch - Foot
};
static constexpr DH_parameter DH_Right_Leg[NUM_JOINTS] = {
    // a      alpha   d       theta
    {0.135f, 0, 0, 0},       // Base - Hip Yaw
    {0.095f, PI / 2, 0, 0},  // Hip Yaw - Hip Roll
    {-0.09f, -PI / 2, 0, 0}, // Hip Roll - Hip Pitch
    {-0.18f, 0, 0, 0},       // Hip Pitch - Knee Pitch
    {-0.38f, 0, 0, 0}        // Knee Pitch - Foot
};
// Joint angle offsets for the robot
static constexpr float LEFT_LEG_HIP_ROLL_OFFSET = PI / 2;
static constexpr float RIGHT_LEG_HIP_ROLL_OFFSET = -PI / 2;

#endif