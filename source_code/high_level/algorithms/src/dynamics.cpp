#include "dynamics.h"
#include "Eigen/Dense"
#include <cmath>

Acceleration Dynamics::computeCoMAccel(const Position &CoM_pos, const Eigen::Vector3d &accel, const Eigen::Vector3d &gyro, const Eigen::Vector3d &gyro_dot)
{
    // Compute the CoM acceleration of the robot using IMU acceleration
    Eigen::Vector3d CoM_to_IMU = {-CoM_pos.x, -CoM_pos.y, -CoM_pos.z};   // Vector from CoM to IMU in world frame

    // CoM acceleration in world frame calculated using rigid body dynamics: a_B = a_A + w_dot x (r_B - r_A) + w x (w x (r_B - r_A))
    Eigen::Vector3d CoM_accel = accel + gyro_dot.cross(CoM_to_IMU) + gyro.cross(gyro.cross(CoM_to_IMU));

    return {CoM_accel(0), CoM_accel(1), CoM_accel(2)};
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