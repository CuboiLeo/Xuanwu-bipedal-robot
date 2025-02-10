#ifndef IMU_H
#define IMU_H

#include <stdint.h>
#include <chrono>
#include "robot_types.h"
#include "Fusion.h"
#include "user_math.h"
#include "Eigen/Dense"
#include <algorithm>

class IMU
{
public:
    IMU();
    Eigen::Vector3d getAccel() const { return accel; };
    Eigen::Vector3d getGyro() const { return gyro; };
    Eigen::Vector3d getGyroDot() const { return gyro_dot; };
    Orientation getEuler() const { return euler_angles; };
    Eigen::Matrix3d getRotationMatrix() const { return rotation_matrix; };

    void setAccel(const Acceleration accel) { this->raw_accel = accel; };
    void setGyro(const Angular_Velocity gyro) { this->raw_gyro = gyro; };
    void computeVel(void);
    void computeGyroDot(void);
    void computeRotationMatrix(void);
    double getTimeElapsed(void);
    void processData(void);

private:
    Acceleration raw_accel = {};        // Acceleration in m/s^2 in IMU frame
    Angular_Velocity raw_gyro = {};     // Angular velocity in rad/s in IMU frame
    Eigen::Vector3d vel;               // Velocity in m/s in world frame
    Eigen::Vector3d accel;            // Acceleration in m/s^2 in world frame
    Eigen::Vector3d gyro = {};         // Angular velocity in rad/s in world frame
    Eigen::Vector3d prev_gyro = {};    // Previous angular velocity in rad/s
    Eigen::Vector3d gyro_dot = {}; // Angular acceleration in rad/s^2

    FusionAhrs ahrs;                 // AHRS algorithm structure
    Orientation euler_angles = {};   // Euler angles in rads for roll, pitch, and yaw
    Eigen::Matrix3d rotation_matrix; // Rotation matrix from base frame to world frame

    double LPF_coeff = 0.01; // Low pass filter coefficient

    std::chrono::high_resolution_clock::time_point last_time;    // Time point of the last IMU update
    std::chrono::high_resolution_clock::time_point current_time; // Time point of the current IMU update
    double delta_t;                                              // Time elapsed since the last IMU update
};

#endif