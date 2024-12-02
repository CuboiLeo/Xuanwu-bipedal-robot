#ifndef ROBOT_CONFIGS_H
#define ROBOT_CONFIGS_H

#include "robot_types.h"
#include "user_math.h"
#include <stdint.h>

// Leg IDs
static constexpr uint8_t LEFT_LEG_ID = 0;
static constexpr uint8_t RIGHT_LEG_ID = 1;

// Lengths of the robot's links in meters
static constexpr float L1 = 0.135f; // x distance from center of center control module to center of hip yaw motor
static constexpr float L2 = 0.12f;  // z distance from center of hip yaw motor to center of hip roll motor
static constexpr float L3 = 0.09f;  // y distance from center of hip roll motor to center of hip pitch motor
static constexpr float L4 = 0.18f;  // z distance from center of hip pitch motor to center of knee pitch motor
static constexpr float L5 = 0.27f;  // z distance from center of knee pitch motor to center of foot

#endif