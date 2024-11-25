#ifndef ROBOT_CONFIGS_H
#define ROBOT_CONFIGS_H

#include "robot_types.h"
#include "user_math.h"
#include <stdint.h>

// Leg IDs
static constexpr uint8_t LEFT_LEG_ID = 0;
static constexpr uint8_t RIGHT_LEG_ID = 1;

// Number of joints per leg, end effector counts
static constexpr int NUM_JOINTS = 5;

// Denavit-Hartenberg parameters for the robot
static constexpr DH_Parameter DH_Left_Leg[NUM_JOINTS] = {
    // a      alpha   d       theta
    {-0.135f, 0, 0, 0},       // Base - Hip Yaw
    {-0.095f, -PI / 2, 0, 0}, // Hip Yaw - Hip Roll
    {-0.09f, -PI / 2, 0, 0},  // Hip Roll - Hip Pitch
    {-0.18f, 0, 0, 0},        // Hip Pitch - Knee Pitch
    {-0.38f, 0, 0, 0}         // Knee Pitch - Foot
};
static constexpr DH_Parameter DH_Right_Leg[NUM_JOINTS] = {
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