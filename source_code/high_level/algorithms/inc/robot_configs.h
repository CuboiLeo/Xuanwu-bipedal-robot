#ifndef ROBOT_CONFIGS_H
#define ROBOT_CONFIGS_H

#include "robot_types.h"
#include "user_math.h"
#include <stdint.h>

// Leg IDs
static constexpr uint8_t LEFT_LEG_ID = 0;
static constexpr uint8_t RIGHT_LEG_ID = 1;

// Lengths between two joints in meters
static constexpr float L1 = 0.135f; // x distance from center of center control module to center of hip yaw motor
static constexpr float L2 = 0.12f;  // z distance from center of hip yaw motor to center of hip roll motor
static constexpr float L3 = 0.09f;  // y distance from center of hip roll motor to center of hip pitch motor
static constexpr float L4 = 0.18f;  // z distance from center of hip pitch motor to center of knee pitch motor
static constexpr float L5 = 0.27f;  // z distance from center of knee pitch motor to center of foot
static constexpr float ROBOT_HEIGHT = L2 + L4 + L5; // Height of the robot from the ground to the center of center control module

// Mass of the robot's links in kg
static constexpr float M1 = 1.73f; // Main control assembly + 2 * hip yaw connector + 2 * hip yaw motors
static constexpr float M2 = 0.55f;  // Hip roll motor + hip roll connector
static constexpr float M3 = 0.38f;  // Hip pitch motor + hip pitch connector
static constexpr float M4 = 0.53f;  // Knee pitch motor + knee pitch connector
static constexpr float M5 = 0.42f;  // Ankle connector + foot connector
static constexpr float MT = M1 + 2*M2 + 2*M3 + 2*M4 + 2*M5; // Total mass of the robot

// Link CoM position offsets in meters (for left leg, right leg is the same but with opposite x sign)
static constexpr Direction_Vector C1 = {0.0f, 0.0f, 0.027f}; // Base frame to link 1 CoM
static constexpr Direction_Vector C2 = {-0.0075f, -0.06f, -0.067f}; // Hip yaw motor frame to link 2 CoM
static constexpr Direction_Vector C3 = {0.0f, 0.084f, 0.0f}; // Hip roll motor frame to link 3 CoM
static constexpr Direction_Vector C4 = {0.0f, 0.0f, -0.142f}; // Hip pitch motor frame to link 4 CoM
static constexpr Direction_Vector C5 = {0.0f, 0.0f, -0.131f}; // Knee pitch motor frame to link 5 CoM
static constexpr float COM_HEIGHT = 0.44f; // Height of the CoM from the ground




#endif