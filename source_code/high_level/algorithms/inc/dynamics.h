#ifndef DYNAMICS_H
#define DYNAMICS_H

#include "stdint.h"
#include "robot_types.h"
#include "robot_configs.h"
#include "Fusion.h"
#include "Eigen/Dense"

class Dynamics
{
public:
    Direction_Vector computeCenterOfMass(const Joint_Angles &joint_angles_left, const Joint_Angles &joint_angles_right, const FusionMatrix &rotation_matrix);
    Direction_Vector computeZeroMomentPoint(const Direction_Vector &accel, const Direction_Vector &gyro, const Direction_Vector &gyro_dot, const FusionMatrix &rotation_matrix);

private:
    Direction_Vector CoM_link1 = {}; // center of mass of the main control assembly + 2 * hip yaw connector + 2 * hip yaw motors
    Direction_Vector CoM_link2 = {}; // center of mass of the left hip roll motor + hip roll connector
    Direction_Vector CoM_link3 = {}; // center of mass of the left hip pitch motor + hip pitch connector
    Direction_Vector CoM_link4 = {}; // center of mass of the left knee pitch motor + knee pitch connector
    Direction_Vector CoM_link5 = {}; // center of mass of the left ankle connector + foot connector
    Direction_Vector CoM_link6 = {}; // center of mass of the right hip roll motor + hip roll connector
    Direction_Vector CoM_link7 = {}; // center of mass of the right hip pitch motor + hip pitch connector
    Direction_Vector CoM_link8 = {}; // center of mass of the right knee pitch motor + knee pitch connector
    Direction_Vector CoM_link9 = {}; // center of mass of the right ankle connector + foot connector

    Direction_Vector CoM_body_frame = {};        // center of mass of the robot in the body frame
    Direction_Vector CoM_world_frame = {};       // center of mass of the robot in the world frame
    Direction_Vector CoM_accel_body_frame = {};  // acceleration of the center of mass of the robot in the body frame
    Direction_Vector CoM_accel_world_frame = {}; // acceleration of the center of mass of the robot in the world frame
    Direction_Vector ZMP_world_frame = {};       // zero moment point of the robot in the world frame
};

#endif // DYNAMICS_H
