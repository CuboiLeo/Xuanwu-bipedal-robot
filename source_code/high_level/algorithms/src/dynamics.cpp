#include "dynamics.h"
#include "Eigen/Dense"
#include <cmath>

Direction_Vector Dynamics::computeCenterOfMass(const Joint_Angles &joint_angles_left, const Joint_Angles &joint_angles_right, const FusionMatrix &rotation_matrix)
{
    // Compute the CoM of center part
    CoM_link1.x = C1.x;
    CoM_link1.y = C1.y;
    CoM_link1.z = C1.z;

    // Compute the center of mass of the left leg parts
    CoM_link2.x = C2.x*cosf(joint_angles_left.hip_yaw) - L1 + C2.y*sinf(joint_angles_left.hip_yaw);
    CoM_link2.y = C2.y*cosf(joint_angles_left.hip_yaw) - C2.x*sinf(joint_angles_left.hip_yaw);
    CoM_link2.z = C2.z;

    CoM_link3.x = C3.y*sinf(joint_angles_left.hip_yaw) - L1 - L3*sinf(joint_angles_left.hip_yaw) + C3.x*cosf(joint_angles_left.hip_yaw)*cosf(joint_angles_left.hip_roll) + C3.z*cosf(joint_angles_left.hip_yaw)*sinf(joint_angles_left.hip_roll);
    CoM_link3.y = C3.y*cosf(joint_angles_left.hip_yaw) - L3*cosf(joint_angles_left.hip_yaw) - C3.x*cosf(joint_angles_left.hip_roll)*sinf(joint_angles_left.hip_yaw) - C3.z*sinf(joint_angles_left.hip_yaw)*sinf(joint_angles_left.hip_roll);
    CoM_link3.z = C3.z*cosf(joint_angles_left.hip_roll) - L2 - C3.x*sinf(joint_angles_left.hip_roll);

    CoM_link4.x = C4.x*cosf(joint_angles_left.hip_yaw)*cosf(joint_angles_left.hip_roll) - L1 + C4.y*cosf(joint_angles_left.hip_pitch)*sinf(joint_angles_left.hip_yaw) - C4.z*sinf(joint_angles_left.hip_yaw)*sinf(joint_angles_left.hip_pitch) + C4.z*cosf(joint_angles_left.hip_yaw)*cosf(joint_angles_left.hip_pitch)*sinf(joint_angles_left.hip_roll) + C4.y*cosf(joint_angles_left.hip_yaw)*sinf(joint_angles_left.hip_roll)*sinf(joint_angles_left.hip_pitch);
    CoM_link4.y = C4.y*cosf(joint_angles_left.hip_yaw)*cosf(joint_angles_left.hip_pitch) - C4.x*cosf(joint_angles_left.hip_roll)*sinf(joint_angles_left.hip_yaw) - C4.z*cosf(joint_angles_left.hip_yaw)*sinf(joint_angles_left.hip_pitch) - C4.z*cosf(joint_angles_left.hip_pitch)*sinf(joint_angles_left.hip_yaw)*sinf(joint_angles_left.hip_roll) - C4.y*sinf(joint_angles_left.hip_yaw)*sinf(joint_angles_left.hip_roll)*sinf(joint_angles_left.hip_pitch);
    CoM_link4.z = C4.z*cosf(joint_angles_left.hip_roll)*cosf(joint_angles_left.hip_pitch) - C4.x*sinf(joint_angles_left.hip_roll) - L2 + C4.y*cosf(joint_angles_left.hip_roll)*sinf(joint_angles_left.hip_pitch);

    CoM_link5.x = C5.x*cosf(joint_angles_left.hip_yaw)*cosf(joint_angles_left.hip_roll) - L1 + L4*sinf(joint_angles_left.hip_yaw)*sinf(joint_angles_left.hip_pitch) + C5.y*cosf(joint_angles_left.hip_pitch)*cosf(joint_angles_left.knee_pitch)*sinf(joint_angles_left.hip_yaw) - L4*cosf(joint_angles_left.hip_yaw)*cosf(joint_angles_left.hip_pitch)*sinf(joint_angles_left.hip_roll) - C5.z*cosf(joint_angles_left.hip_pitch)*sinf(joint_angles_left.hip_yaw)*sinf(joint_angles_left.knee_pitch) - C5.z*cosf(joint_angles_left.knee_pitch)*sinf(joint_angles_left.hip_yaw)*sinf(joint_angles_left.hip_pitch) - C5.y*sinf(joint_angles_left.hip_yaw)*sinf(joint_angles_left.hip_pitch)*sinf(joint_angles_left.knee_pitch) + C5.z*cosf(joint_angles_left.hip_yaw)*cosf(joint_angles_left.hip_pitch)*cosf(joint_angles_left.knee_pitch)*sinf(joint_angles_left.hip_roll) + C5.y*cosf(joint_angles_left.hip_yaw)*cosf(joint_angles_left.hip_pitch)*sinf(joint_angles_left.hip_roll)*sinf(joint_angles_left.knee_pitch) + C5.y*cosf(joint_angles_left.hip_yaw)*cosf(joint_angles_left.knee_pitch)*sinf(joint_angles_left.hip_roll)*sinf(joint_angles_left.hip_pitch) - C5.z*cosf(joint_angles_left.hip_yaw)*sinf(joint_angles_left.hip_roll)*sinf(joint_angles_left.hip_pitch)*sinf(joint_angles_left.knee_pitch);
    CoM_link5.y = L4*cosf(joint_angles_left.hip_yaw)*sinf(joint_angles_left.hip_pitch) - C5.x*cosf(joint_angles_left.hip_roll)*sinf(joint_angles_left.hip_yaw) + C5.y*cosf(joint_angles_left.hip_yaw)*cosf(joint_angles_left.hip_pitch)*cosf(joint_angles_left.knee_pitch) - C5.z*cosf(joint_angles_left.hip_yaw)*cosf(joint_angles_left.hip_pitch)*sinf(joint_angles_left.knee_pitch) - C5.z*cosf(joint_angles_left.hip_yaw)*cosf(joint_angles_left.knee_pitch)*sinf(joint_angles_left.hip_pitch) - C5.y*cosf(joint_angles_left.hip_yaw)*sinf(joint_angles_left.hip_pitch)*sinf(joint_angles_left.knee_pitch) + L4*cosf(joint_angles_left.hip_pitch)*sinf(joint_angles_left.hip_yaw)*sinf(joint_angles_left.hip_roll) - C5.z*cosf(joint_angles_left.hip_pitch)*cosf(joint_angles_left.knee_pitch)*sinf(joint_angles_left.hip_yaw)*sinf(joint_angles_left.hip_roll) - C5.y*cosf(joint_angles_left.hip_pitch)*sinf(joint_angles_left.hip_yaw)*sinf(joint_angles_left.hip_roll)*sinf(joint_angles_left.knee_pitch) - C5.y*cosf(joint_angles_left.knee_pitch)*sinf(joint_angles_left.hip_yaw)*sinf(joint_angles_left.hip_roll)*sinf(joint_angles_left.hip_pitch) + C5.z*sinf(joint_angles_left.hip_yaw)*sinf(joint_angles_left.hip_roll)*sinf(joint_angles_left.hip_pitch)*sinf(joint_angles_left.knee_pitch);
    CoM_link5.z = C5.z*cosf(joint_angles_left.hip_roll)*cosf(joint_angles_left.hip_pitch)*cosf(joint_angles_left.knee_pitch) - C5.x*sinf(joint_angles_left.hip_roll) - L4*cosf(joint_angles_left.hip_roll)*cosf(joint_angles_left.hip_pitch) - L2 + C5.y*cosf(joint_angles_left.hip_roll)*cosf(joint_angles_left.hip_pitch)*sinf(joint_angles_left.knee_pitch) + C5.y*cosf(joint_angles_left.hip_roll)*cosf(joint_angles_left.knee_pitch)*sinf(joint_angles_left.hip_pitch) - C5.z*cosf(joint_angles_left.hip_roll)*sinf(joint_angles_left.hip_pitch)*sinf(joint_angles_left.knee_pitch);

    // Compute the center of mass of the right leg parts
    CoM_link6.x = L1 - C2.x*cosf(joint_angles_right.hip_yaw) + C2.y*sinf(joint_angles_right.hip_yaw);
    CoM_link6.y = C2.y*cosf(joint_angles_right.hip_yaw) + C2.x*sinf(joint_angles_right.hip_yaw);
    CoM_link6.z = C2.z;

    CoM_link7.x = L1 + C3.y*sinf(joint_angles_right.hip_yaw) - L3*sinf(joint_angles_right.hip_yaw) + C3.x*cosf(joint_angles_right.hip_yaw)*cosf(joint_angles_right.hip_roll) + C3.z*cosf(joint_angles_right.hip_yaw)*sinf(joint_angles_right.hip_roll);
    CoM_link7.y = C3.y*cosf(joint_angles_right.hip_yaw) - L3*cosf(joint_angles_right.hip_yaw) - C3.x*cosf(joint_angles_right.hip_roll)*sinf(joint_angles_right.hip_yaw) - C3.z*sinf(joint_angles_right.hip_yaw)*sinf(joint_angles_right.hip_roll);
    CoM_link7.z = C3.z*cosf(joint_angles_right.hip_roll) - L2 - C3.x*sinf(joint_angles_right.hip_roll);

    CoM_link8.x = L1 + C4.x*cosf(joint_angles_right.hip_yaw)*cosf(joint_angles_right.hip_roll) + C4.y*cosf(joint_angles_right.hip_pitch)*sinf(joint_angles_right.hip_yaw) - C4.z*sinf(joint_angles_right.hip_yaw)*sinf(joint_angles_right.hip_pitch) + C4.z*cosf(joint_angles_right.hip_yaw)*cosf(joint_angles_right.hip_pitch)*sinf(joint_angles_right.hip_roll) + C4.y*cosf(joint_angles_right.hip_yaw)*sinf(joint_angles_right.hip_roll)*sinf(joint_angles_right.hip_pitch);
    CoM_link8.y = C4.y*cosf(joint_angles_right.hip_yaw)*cosf(joint_angles_right.hip_pitch) - C4.x*cosf(joint_angles_right.hip_roll)*sinf(joint_angles_right.hip_yaw) - C4.z*cosf(joint_angles_right.hip_yaw)*sinf(joint_angles_right.hip_pitch) - C4.z*cosf(joint_angles_right.hip_pitch)*sinf(joint_angles_right.hip_yaw)*sinf(joint_angles_right.hip_roll) - C4.y*sinf(joint_angles_right.hip_yaw)*sinf(joint_angles_right.hip_roll)*sinf(joint_angles_right.hip_pitch);
    CoM_link8.z = C4.z*cosf(joint_angles_right.hip_roll)*cosf(joint_angles_right.hip_pitch) - C4.x*sinf(joint_angles_right.hip_roll) - L2 + C4.y*cosf(joint_angles_right.hip_roll)*sinf(joint_angles_right.hip_pitch);

    CoM_link9.x = L1 + C5.x*cosf(joint_angles_right.hip_yaw)*cosf(joint_angles_right.hip_roll) + L4*sinf(joint_angles_right.hip_yaw)*sinf(joint_angles_right.hip_pitch) + C5.y*cosf(joint_angles_right.hip_pitch)*cosf(joint_angles_right.knee_pitch)*sinf(joint_angles_right.hip_yaw) - L4*cosf(joint_angles_right.hip_yaw)*cosf(joint_angles_right.hip_pitch)*sinf(joint_angles_right.hip_roll) - C5.z*cosf(joint_angles_right.hip_pitch)*sinf(joint_angles_right.hip_yaw)*sinf(joint_angles_right.knee_pitch) - C5.z*cosf(joint_angles_right.knee_pitch)*sinf(joint_angles_right.hip_yaw)*sinf(joint_angles_right.hip_pitch) - C5.y*sinf(joint_angles_right.hip_yaw)*sinf(joint_angles_right.hip_pitch)*sinf(joint_angles_right.knee_pitch) + C5.z*cosf(joint_angles_right.hip_yaw)*cosf(joint_angles_right.hip_pitch)*cosf(joint_angles_right.knee_pitch)*sinf(joint_angles_right.hip_roll) + C5.y*cosf(joint_angles_right.hip_yaw)*cosf(joint_angles_right.hip_pitch)*sinf(joint_angles_right.hip_roll)*sinf(joint_angles_right.knee_pitch) + C5.y*cosf(joint_angles_right.hip_yaw)*cosf(joint_angles_right.knee_pitch)*sinf(joint_angles_right.hip_roll)*sinf(joint_angles_right.hip_pitch) - C5.z*cosf(joint_angles_right.hip_yaw)*sinf(joint_angles_right.hip_roll)*sinf(joint_angles_right.hip_pitch)*sinf(joint_angles_right.knee_pitch);
    CoM_link9.y = L4*cosf(joint_angles_right.hip_yaw)*sinf(joint_angles_right.hip_pitch) - C5.x*cosf(joint_angles_right.hip_roll)*sinf(joint_angles_right.hip_yaw) + C5.y*cosf(joint_angles_right.hip_yaw)*cosf(joint_angles_right.hip_pitch)*cosf(joint_angles_right.knee_pitch) - C5.z*cosf(joint_angles_right.hip_yaw)*cosf(joint_angles_right.hip_pitch)*sinf(joint_angles_right.knee_pitch) - C5.z*cosf(joint_angles_right.hip_yaw)*cosf(joint_angles_right.knee_pitch)*sinf(joint_angles_right.hip_pitch) - C5.y*cosf(joint_angles_right.hip_yaw)*sinf(joint_angles_right.hip_pitch)*sinf(joint_angles_right.knee_pitch) + L4*cosf(joint_angles_right.hip_pitch)*sinf(joint_angles_right.hip_yaw)*sinf(joint_angles_right.hip_roll) - C5.z*cosf(joint_angles_right.hip_pitch)*cosf(joint_angles_right.knee_pitch)*sinf(joint_angles_right.hip_yaw)*sinf(joint_angles_right.hip_roll) - C5.y*cosf(joint_angles_right.hip_pitch)*sinf(joint_angles_right.hip_yaw)*sinf(joint_angles_right.hip_roll)*sinf(joint_angles_right.knee_pitch) - C5.y*cosf(joint_angles_right.knee_pitch)*sinf(joint_angles_right.hip_yaw)*sinf(joint_angles_right.hip_roll)*sinf(joint_angles_right.hip_pitch) + C5.z*sinf(joint_angles_right.hip_yaw)*sinf(joint_angles_right.hip_roll)*sinf(joint_angles_right.hip_pitch)*sinf(joint_angles_right.knee_pitch);
    CoM_link9.z = C5.z*cosf(joint_angles_right.hip_roll)*cosf(joint_angles_right.hip_pitch)*cosf(joint_angles_right.knee_pitch) - C5.x*sinf(joint_angles_right.hip_roll) - L4*cosf(joint_angles_right.hip_roll)*cosf(joint_angles_right.hip_pitch) - L2 + C5.y*cosf(joint_angles_right.hip_roll)*cosf(joint_angles_right.hip_pitch)*sinf(joint_angles_right.knee_pitch) + C5.y*cosf(joint_angles_right.hip_roll)*cosf(joint_angles_right.knee_pitch)*sinf(joint_angles_right.hip_pitch) - C5.z*cosf(joint_angles_right.hip_roll)*sinf(joint_angles_right.hip_pitch)*sinf(joint_angles_right.knee_pitch);

    // Compute the center of mass of the robot
    CoM_body_frame.x = (CoM_link1.x * M1 + CoM_link2.x * M2 + CoM_link3.x * M3 + CoM_link4.x * M4 + CoM_link5.x * M5 + CoM_link6.x * M2 + CoM_link7.x * M3 + CoM_link8.x * M4 + CoM_link9.x * M5) / MT;
    CoM_body_frame.y = (CoM_link1.y * M1 + CoM_link2.y * M2 + CoM_link3.y * M3 + CoM_link4.y * M4 + CoM_link5.y * M5 + CoM_link6.y * M2 + CoM_link7.y * M3 + CoM_link8.y * M4 + CoM_link9.y * M5) / MT;
    CoM_body_frame.z = (CoM_link1.z * M1 + CoM_link2.z * M2 + CoM_link3.z * M3 + CoM_link4.z * M4 + CoM_link5.z * M5 + CoM_link6.z * M2 + CoM_link7.z * M3 + CoM_link8.z * M4 + CoM_link9.z * M5) / MT;

    // Rotate the center of mass of the robot from body frame to world frame
    CoM_world_frame.x = (CoM_body_frame.x * rotation_matrix.array[0][0] + CoM_body_frame.y * rotation_matrix.array[0][1] + CoM_body_frame.z * rotation_matrix.array[0][2]);
    CoM_world_frame.y = (CoM_body_frame.x * rotation_matrix.array[1][0] + CoM_body_frame.y * rotation_matrix.array[1][1] + CoM_body_frame.z * rotation_matrix.array[1][2]);
    CoM_world_frame.z = CoM_body_frame.x * rotation_matrix.array[2][0] + CoM_body_frame.y * rotation_matrix.array[2][1] + CoM_body_frame.z * rotation_matrix.array[2][2];

    return CoM_world_frame;
}

Direction_Vector Dynamics::computeZeroMomentPoint(const Direction_Vector &accel, const Direction_Vector &gyro, const Direction_Vector &gyro_dot, const FusionMatrix &rotation_matrix)
{
    // Compute the CoM acceleration of the robot using IMU acceleration
    Eigen::Vector3f v_CoM_accel_body_frame = {CoM_accel_body_frame.x, CoM_accel_body_frame.y, CoM_accel_body_frame.z};            // CoM acceleration in body frame
    Eigen::Vector3f v_IMU_accel = {accel.x, accel.y, accel.z};                                                                    // IMU acceleration in world frame
    Eigen::Vector3f v_IMU_gyro = {gyro.x, gyro.y, gyro.z};                                                                        // IMU angular velocity in world frame
    Eigen::Vector3f v_IMU_gyro_dot = {gyro_dot.x, gyro_dot.y, gyro_dot.z};                                                        // IMU angular acceleration in world frame
    Eigen::Vector3f v_CoM_to_IMU = {CoM_body_frame.x, CoM_body_frame.y, CoM_body_frame.z};                                        // Vector from CoM to IMU in body frame
    
    v_CoM_accel_body_frame = v_IMU_accel + v_IMU_gyro_dot.cross(v_CoM_to_IMU) + v_IMU_gyro.cross(v_IMU_gyro.cross(v_CoM_to_IMU)); // CoM acceleration in body frame calculated using rigid body dynamics
    
    CoM_accel_body_frame = {v_CoM_accel_body_frame(0), v_CoM_accel_body_frame(1), v_CoM_accel_body_frame(2)};

    // Rotate the CoM acceleration from body frame to world frame
    Eigen::Matrix3f m_rotation_matrix;
    m_rotation_matrix << rotation_matrix.array[0][0], rotation_matrix.array[0][1], rotation_matrix.array[0][2],
        rotation_matrix.array[1][0], rotation_matrix.array[1][1], rotation_matrix.array[1][2],
        rotation_matrix.array[2][0], rotation_matrix.array[2][1], rotation_matrix.array[2][2];

    Eigen::Vector3f v_CoM_accel_world_frame = m_rotation_matrix * v_CoM_accel_body_frame;

    CoM_accel_world_frame = {v_CoM_accel_world_frame(0), v_CoM_accel_world_frame(1), v_CoM_accel_world_frame(2)};

    // Compute the zero moment point of the robot
    ZMP_world_frame.x = CoM_world_frame.x - ((ROBOT_HEIGHT + CoM_world_frame.z) / GRAVITY) * CoM_accel_world_frame.x;
    ZMP_world_frame.y = CoM_world_frame.y - ((ROBOT_HEIGHT + CoM_world_frame.z) / GRAVITY) * CoM_accel_world_frame.y;
    ZMP_world_frame.z = 0.0f;

    return ZMP_world_frame;
}