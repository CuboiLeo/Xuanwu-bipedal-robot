#include "dynamics.h"
#include "Eigen/Dense"
#include <cmath>

Joint_Torques Dynamics::computeGRFTorques(const Wrench &GRF, const Joint_Angles &angles, const uint8_t &leg_id)
{
    Eigen::MatrixXd Jacobian(6, 5); // Initialize the spatial Jacobian
    switch (leg_id)
    {
    case LEFT_LEG_ID:
        Jacobian << 0, L2 * cos(angles.hip_yaw), (-L2) * cos(angles.hip_roll) * sin(angles.hip_yaw), (-L2) * cos(angles.hip_roll) * sin(angles.hip_yaw) - L4 * (cos(angles.hip_pitch) * sin(angles.hip_yaw) + cos(angles.hip_yaw) * sin(angles.hip_roll) * sin(angles.hip_pitch)), (-L2) * cos(angles.hip_roll) * sin(angles.hip_yaw) + sin(angles.hip_pitch) * ((-cos(angles.hip_yaw)) * (L4 + L5 * cos(angles.knee_pitch)) * sin(angles.hip_roll) + L5 * sin(angles.hip_yaw) * sin(angles.knee_pitch)) - cos(angles.hip_pitch) * ((L4 + L5 * cos(angles.knee_pitch)) * sin(angles.hip_yaw) + L5 * cos(angles.hip_yaw) * sin(angles.hip_roll) * sin(angles.knee_pitch)),
            -L1, (-L2) * sin(angles.hip_yaw), (-L2) * cos(angles.hip_yaw) * cos(angles.hip_roll) - L1 * sin(angles.hip_roll), (-cos(angles.hip_yaw)) * (L2 * cos(angles.hip_roll) + L4 * cos(angles.hip_pitch)) + sin(angles.hip_roll) * (-L1 + L4 * sin(angles.hip_yaw) * sin(angles.hip_pitch)), (-cos(angles.hip_yaw)) * (L2 * cos(angles.hip_roll) + L4 * cos(angles.hip_pitch) + L5 * cos(angles.hip_pitch + angles.knee_pitch)) + sin(angles.hip_roll) * (-L1 + sin(angles.hip_yaw) * (L4 * sin(angles.hip_pitch) + L5 * sin(angles.hip_pitch + angles.knee_pitch))),
            0, (-L1) * cos(angles.hip_yaw), L1 * cos(angles.hip_roll) * sin(angles.hip_yaw), cos(angles.hip_roll) * (L1 * sin(angles.hip_yaw) - L4 * sin(angles.hip_pitch)), cos(angles.hip_roll) * (L1 * sin(angles.hip_yaw) - L4 * sin(angles.hip_pitch) - L5 * sin(angles.hip_pitch + angles.knee_pitch)),
            0, sin(angles.hip_yaw), cos(angles.hip_yaw) * cos(angles.hip_roll), cos(angles.hip_yaw) * cos(angles.hip_roll), cos(angles.hip_yaw) * cos(angles.hip_roll),
            0, cos(angles.hip_yaw), (-cos(angles.hip_roll)) * sin(angles.hip_yaw), (-cos(angles.hip_roll)) * sin(angles.hip_yaw), (-cos(angles.hip_roll)) * sin(angles.hip_yaw),
            -1, 0, -sin(angles.hip_roll), -sin(angles.hip_roll), -sin(angles.hip_roll);
        break;
    case RIGHT_LEG_ID:
        Jacobian << 0, L2 * cos(angles.hip_yaw), (-L2) * cos(angles.hip_roll) * sin(angles.hip_yaw), (-L2) * cos(angles.hip_roll) * sin(angles.hip_yaw) - L4 * (cos(angles.hip_pitch) * sin(angles.hip_yaw) + cos(angles.hip_yaw) * sin(angles.hip_roll) * sin(angles.hip_pitch)), (-L2) * cos(angles.hip_roll) * sin(angles.hip_yaw) + sin(angles.hip_pitch) * ((-cos(angles.hip_yaw)) * (L4 + L5 * cos(angles.knee_pitch)) * sin(angles.hip_roll) + L5 * sin(angles.hip_yaw) * sin(angles.knee_pitch)) - cos(angles.hip_pitch) * ((L4 + L5 * cos(angles.knee_pitch)) * sin(angles.hip_yaw) + L5 * cos(angles.hip_yaw) * sin(angles.hip_roll) * sin(angles.knee_pitch)),
            L1, (-L2) * sin(angles.hip_yaw), (-L2) * cos(angles.hip_yaw) * cos(angles.hip_roll) + L1 * sin(angles.hip_roll), (-cos(angles.hip_yaw)) * (L2 * cos(angles.hip_roll) + L4 * cos(angles.hip_pitch)) + sin(angles.hip_roll) * (L1 + L4 * sin(angles.hip_yaw) * sin(angles.hip_pitch)), (-cos(angles.hip_yaw)) * (L2 * cos(angles.hip_roll) + L4 * cos(angles.hip_pitch) + L5 * cos(angles.hip_pitch + angles.knee_pitch)) + sin(angles.hip_roll) * (L1 + sin(angles.hip_yaw) * (L4 * sin(angles.hip_pitch) + L5 * sin(angles.hip_pitch + angles.knee_pitch))),
            0, L1 * cos(angles.hip_yaw), (-L1) * cos(angles.hip_roll) * sin(angles.hip_yaw), (-cos(angles.hip_roll)) * (L1 * sin(angles.hip_yaw) + L4 * sin(angles.hip_pitch)), (-cos(angles.hip_roll)) * (L1 * sin(angles.hip_yaw) + L4 * sin(angles.hip_pitch) + L5 * sin(angles.hip_pitch + angles.knee_pitch)),
            0, sin(angles.hip_yaw), cos(angles.hip_yaw) * cos(angles.hip_roll), cos(angles.hip_yaw) * cos(angles.hip_roll), cos(angles.hip_yaw) * cos(angles.hip_roll),
            0, cos(angles.hip_yaw), (-cos(angles.hip_roll)) * sin(angles.hip_yaw), (-cos(angles.hip_roll)) * sin(angles.hip_yaw), (-cos(angles.hip_roll)) * sin(angles.hip_yaw),
            -1, 0, -sin(angles.hip_roll), -sin(angles.hip_roll), -sin(angles.hip_roll);
        break;
    }
    // Compute the joint torques
    Eigen::VectorXd torques(5);
    Eigen::VectorXd Ground_Reaction_Forces(6);
    Ground_Reaction_Forces << GRF.force.x, GRF.force.y, GRF.force.z, GRF.torque.x, GRF.torque.y, GRF.torque.z;
    torques = Jacobian.transpose() * Ground_Reaction_Forces;

    return {torques(0), torques(1), torques(2), torques(3), torques(4)};
}

Wrench Dynamics::computeFootWrench(const Joint_Torques &torques, const Joint_Angles &angles, const uint8_t &leg_id)
{
    Eigen::MatrixXd Jacobian(6, 5); // Initialize the spatial Jacobian
    switch (leg_id)
    {
    case LEFT_LEG_ID:
        Jacobian << 0, L2 * cos(angles.hip_yaw), (-L2) * cos(angles.hip_roll) * sin(angles.hip_yaw), (-L2) * cos(angles.hip_roll) * sin(angles.hip_yaw) - L4 * (cos(angles.hip_pitch) * sin(angles.hip_yaw) + cos(angles.hip_yaw) * sin(angles.hip_roll) * sin(angles.hip_pitch)), (-L2) * cos(angles.hip_roll) * sin(angles.hip_yaw) + sin(angles.hip_pitch) * ((-cos(angles.hip_yaw)) * (L4 + L5 * cos(angles.knee_pitch)) * sin(angles.hip_roll) + L5 * sin(angles.hip_yaw) * sin(angles.knee_pitch)) - cos(angles.hip_pitch) * ((L4 + L5 * cos(angles.knee_pitch)) * sin(angles.hip_yaw) + L5 * cos(angles.hip_yaw) * sin(angles.hip_roll) * sin(angles.knee_pitch)),
            -L1, (-L2) * sin(angles.hip_yaw), (-L2) * cos(angles.hip_yaw) * cos(angles.hip_roll) - L1 * sin(angles.hip_roll), (-cos(angles.hip_yaw)) * (L2 * cos(angles.hip_roll) + L4 * cos(angles.hip_pitch)) + sin(angles.hip_roll) * (-L1 + L4 * sin(angles.hip_yaw) * sin(angles.hip_pitch)), (-cos(angles.hip_yaw)) * (L2 * cos(angles.hip_roll) + L4 * cos(angles.hip_pitch) + L5 * cos(angles.hip_pitch + angles.knee_pitch)) + sin(angles.hip_roll) * (-L1 + sin(angles.hip_yaw) * (L4 * sin(angles.hip_pitch) + L5 * sin(angles.hip_pitch + angles.knee_pitch))),
            0, (-L1) * cos(angles.hip_yaw), L1 * cos(angles.hip_roll) * sin(angles.hip_yaw), cos(angles.hip_roll) * (L1 * sin(angles.hip_yaw) - L4 * sin(angles.hip_pitch)), cos(angles.hip_roll) * (L1 * sin(angles.hip_yaw) - L4 * sin(angles.hip_pitch) - L5 * sin(angles.hip_pitch + angles.knee_pitch)),
            0, sin(angles.hip_yaw), cos(angles.hip_yaw) * cos(angles.hip_roll), cos(angles.hip_yaw) * cos(angles.hip_roll), cos(angles.hip_yaw) * cos(angles.hip_roll),
            0, cos(angles.hip_yaw), (-cos(angles.hip_roll)) * sin(angles.hip_yaw), (-cos(angles.hip_roll)) * sin(angles.hip_yaw), (-cos(angles.hip_roll)) * sin(angles.hip_yaw),
            -1, 0, -sin(angles.hip_roll), -sin(angles.hip_roll), -sin(angles.hip_roll);
        break;
    case RIGHT_LEG_ID:
        Jacobian << 0, L2 * cos(angles.hip_yaw), (-L2) * cos(angles.hip_roll) * sin(angles.hip_yaw), (-L2) * cos(angles.hip_roll) * sin(angles.hip_yaw) - L4 * (cos(angles.hip_pitch) * sin(angles.hip_yaw) + cos(angles.hip_yaw) * sin(angles.hip_roll) * sin(angles.hip_pitch)), (-L2) * cos(angles.hip_roll) * sin(angles.hip_yaw) + sin(angles.hip_pitch) * ((-cos(angles.hip_yaw)) * (L4 + L5 * cos(angles.knee_pitch)) * sin(angles.hip_roll) + L5 * sin(angles.hip_yaw) * sin(angles.knee_pitch)) - cos(angles.hip_pitch) * ((L4 + L5 * cos(angles.knee_pitch)) * sin(angles.hip_yaw) + L5 * cos(angles.hip_yaw) * sin(angles.hip_roll) * sin(angles.knee_pitch)),
            L1, (-L2) * sin(angles.hip_yaw), (-L2) * cos(angles.hip_yaw) * cos(angles.hip_roll) + L1 * sin(angles.hip_roll), (-cos(angles.hip_yaw)) * (L2 * cos(angles.hip_roll) + L4 * cos(angles.hip_pitch)) + sin(angles.hip_roll) * (L1 + L4 * sin(angles.hip_yaw) * sin(angles.hip_pitch)), (-cos(angles.hip_yaw)) * (L2 * cos(angles.hip_roll) + L4 * cos(angles.hip_pitch) + L5 * cos(angles.hip_pitch + angles.knee_pitch)) + sin(angles.hip_roll) * (L1 + sin(angles.hip_yaw) * (L4 * sin(angles.hip_pitch) + L5 * sin(angles.hip_pitch + angles.knee_pitch))),
            0, L1 * cos(angles.hip_yaw), (-L1) * cos(angles.hip_roll) * sin(angles.hip_yaw), (-cos(angles.hip_roll)) * (L1 * sin(angles.hip_yaw) + L4 * sin(angles.hip_pitch)), (-cos(angles.hip_roll)) * (L1 * sin(angles.hip_yaw) + L4 * sin(angles.hip_pitch) + L5 * sin(angles.hip_pitch + angles.knee_pitch)),
            0, sin(angles.hip_yaw), cos(angles.hip_yaw) * cos(angles.hip_roll), cos(angles.hip_yaw) * cos(angles.hip_roll), cos(angles.hip_yaw) * cos(angles.hip_roll),
            0, cos(angles.hip_yaw), (-cos(angles.hip_roll)) * sin(angles.hip_yaw), (-cos(angles.hip_roll)) * sin(angles.hip_yaw), (-cos(angles.hip_roll)) * sin(angles.hip_yaw),
            -1, 0, -sin(angles.hip_roll), -sin(angles.hip_roll), -sin(angles.hip_roll);
        break;
    }
    Eigen::VectorXd joint_torques(5);
    joint_torques << torques.hip_yaw, torques.hip_roll, torques.hip_pitch, torques.knee_pitch, torques.ankle_pitch;
    Eigen::VectorXd foot_wrench(6);
    foot_wrench = Jacobian * (Jacobian.transpose() * Jacobian).inverse() * joint_torques;

    return {foot_wrench(0), foot_wrench(1), foot_wrench(2), foot_wrench(3), foot_wrench(4), foot_wrench(5)};
}

Acceleration Dynamics::computeCoMAccel(const Position &CoM_pos, const Eigen::Vector3d &accel, const Eigen::Vector3d &gyro, const Eigen::Vector3d &gyro_dot)
{
    // Compute the CoM acceleration of the robot using IMU acceleration
    Eigen::Vector3d CoM_to_IMU = {-CoM_pos.x, -CoM_pos.y, -CoM_pos.z}; // Vector from CoM to IMU in world frame

    // CoM acceleration in world frame calculated using rigid body dynamics: a_B = a_A + w_dot x (r_B - r_A) + w x (w x (r_B - r_A))
    Eigen::Vector3d CoM_accel = accel + gyro_dot.cross(CoM_to_IMU) + gyro.cross(gyro.cross(CoM_to_IMU));

    return {CoM_accel(0), CoM_accel(1), CoM_accel(2)};
}

Position Dynamics::computeZMPPos(const Position &CoM, const Acceleration &CoM_accel)
{
    ZMP_pos;

    // Compute the zero moment point of the robot
    ZMP_pos.x = filter_coeff * (CoM.x - (COM_HEIGHT / GRAVITY) * CoM_accel.x) + (1 - filter_coeff) * ZMP_pos.x;
    ZMP_pos.y = filter_coeff * (CoM.y - (COM_HEIGHT / GRAVITY) * CoM_accel.y) + (1 - filter_coeff) * ZMP_pos.y;
    ZMP_pos.z = 0.0f;

    return ZMP_pos;
}