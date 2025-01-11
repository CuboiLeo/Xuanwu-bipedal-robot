#include "dynamics.h"
#include "Eigen/Dense"
#include <cmath>

Position Dynamics::computeCoMAccel(const Position &CoM_pos, const Acceleration &accel, const Angular_Velocity &gyro, const Angular_Acceleration &gyro_dot)
{
    // Compute the CoM acceleration of the robot using IMU acceleration
    Eigen::Vector3d v_IMU_accel = {accel.x, accel.y, accel.z};             // IMU acceleration in world frame
    Eigen::Vector3d v_IMU_gyro = {gyro.x, gyro.y, gyro.z};                 // IMU angular velocity in world frame
    Eigen::Vector3d v_IMU_gyro_dot = {gyro_dot.x, gyro_dot.y, gyro_dot.z}; // IMU angular acceleration in world frame
    Eigen::Vector3d v_CoM_to_IMU = {-CoM_pos.x, -CoM_pos.y, -CoM_pos.z};   // Vector from CoM to IMU in world frame

    // CoM acceleration in world frame calculated using rigid body dynamics: a_B = a_A + w_dot x (r_B - r_A) + w x (w x (r_B - r_A))
    Eigen::Vector3d v_CoM_accel = v_IMU_accel + v_IMU_gyro_dot.cross(v_CoM_to_IMU) + v_IMU_gyro.cross(v_IMU_gyro.cross(v_CoM_to_IMU));
    Acceleration CoM_accel = {v_CoM_accel(0), v_CoM_accel(1), v_CoM_accel(2)};

    return CoM_accel;
}

Position Dynamics::computeZMPPos(const Position &CoM, const Acceleration &CoM_accel)
{
    Position ZMP_pos;

    // Compute the zero moment point of the robot
    ZMP_pos.x = CoM.x - (COM_HEIGHT / GRAVITY) * CoM_accel.x;
    ZMP_pos.y = CoM.y - (COM_HEIGHT / GRAVITY) * CoM_accel.y;
    ZMP_pos.z = 0.0f;

    return ZMP_pos;
}