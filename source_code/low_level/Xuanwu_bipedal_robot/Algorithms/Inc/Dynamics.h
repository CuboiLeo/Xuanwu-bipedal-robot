#ifndef DYNAMICS_H
#define DYNAMICS_H

#include "stdint.h"
#include "Robot_Types.h"
#include "Robot.h"
#include "Fusion.h"

class Dynamics
{
public:
    Dynamics();
    Direction_Vector computeCenterOfMass(const Joint_Angle &joint_angles_left, const Joint_Angle &joint_angles_right, const FusionMatrix &rotation_matrix);
    Direction_Vector computeZeroMomentPoint(const FusionVector &accel, const FusionVector &gyro, const FusionVector &gyro_dot, const FusionMatrix &rotation_matrix);

private:
    // Masses of the robot parts (kg)
    static constexpr float MASS_CENTER_PART = 1.68f;      // mass of center connector module + battery + electronics + Hip Yaw motors
    static constexpr float MASS_HIP_ROLL_PART = 0.442f;   // mass of Hip Roll motors
    static constexpr float MASS_HIP_PITCH_PART = 0.442f;  // mass of Hip Pitch motors
    static constexpr float MASS_KNEE_PITCH_PART = 0.562f; // mass of Knee Pitch motors
    static constexpr float MASS_FOOT_PART = 0.483f;       // mass of foot

    // Center of mass positions offsets of the parts (m)
    static constexpr float COM_CENTER_Z_OFFSET = -0.03f;    // distance in z from the center connector module coordinate to the center of mass of the center part
    static constexpr float COM_HIP_ROLL_X_OFFSET = 0.015f;  // distance in x from the center connector module coordinate to the center of mass of the Hip Roll part
    static constexpr float COM_HIP_PITCH_Z_OFFSET = 0.015f; // distance in z from the center connector module coordinate to the center of mass of the Hip Pitch part
    static constexpr float COM_KNEE_PITCH_Z_OFFSET = 0.05f; // distance in z from the center connector module coordinate to the center of mass of the Knee Pitch part
    static constexpr float COM_FOOT_Z_OFFSET = 0.24f;       // distance in z from the center connector module coordinate to the center of mass of the foot

    // Total height of the robot (m)
    static constexpr float ROBOT_HEIGHT = 0.65f; // height of the robot from the ground to the center of mass of center connector module (IMU)

    Direction_Vector CoM_center_part;           // center of mass of the center part
    Direction_Vector CoM_left_hip_roll_part;    // center of mass of the left hip roll part
    Direction_Vector CoM_left_hip_pitch_part;   // center of mass of the left hip pitch part
    Direction_Vector CoM_left_knee_pitch_part;  // center of mass of the left knee pitch part
    Direction_Vector CoM_left_foot_part;        // center of mass of the left foot part
    Direction_Vector CoM_right_hip_roll_part;   // center of mass of the right hip roll part
    Direction_Vector CoM_right_hip_pitch_part;  // center of mass of the right hip pitch part
    Direction_Vector CoM_right_knee_pitch_part; // center of mass of the right knee pitch part
    Direction_Vector CoM_right_foot_part;       // center of mass of the right foot part

    Direction_Vector CoM_body_frame; // center of mass of the robot in the body frame
    Direction_Vector CoM_world_frame; // center of mass of the robot in the world frame
    Direction_Vector CoM_accel_body_frame; // acceleration of the center of mass of the robot in the body frame
    Direction_Vector CoM_accel_world_frame; // acceleration of the center of mass of the robot in the world frame
    Direction_Vector ZMP_world_frame; // zero moment point of the robot in the world frame
};

#endif // DYNAMICS_H
