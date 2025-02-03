#ifndef IMU_H
#define IMU_H

#include <stdint.h>
#include <chrono>
#include "robot_types.h"
#include "Fusion.h"
#include "user_math.h"
#include "Eigen/Dense"

class IMU
{
public:
    IMU();
    Acceleration getAccel() const { return accel; };
    Angular_Velocity getGyro() const { return gyro; };
    Angular_Acceleration getGyroDot() const { return gyro_dot; };
    Orientation getEuler() const { return euler_angles; };
    Eigen::Matrix3d getRotationMatrix() const { return rotation_matrix; };

    void setAccel(const Acceleration accel) { this->accel = accel; };
    void setGyro(const Angular_Velocity gyro) { this->gyro = gyro; };
    void computeGyroDot(void);
    void computeRotationMatrix(void);
    double getTimeElapsed(void);
    void processData(void);

private:
    Acceleration accel = {};            // Acceleration in m/s^2
    Angular_Velocity gyro = {};         // Angular velocity in rad/s
    Angular_Velocity prev_gyro = {};    // Previous angular velocity in rad/s
    Angular_Acceleration gyro_dot = {}; // Angular acceleration in rad/s^2

    FusionAhrs ahrs;                 // AHRS algorithm structure
    Orientation euler_angles = {};   // Euler angles in rads for roll, pitch, and yaw
    Eigen::Matrix3d rotation_matrix; // Rotation matrix from base frame to world frame

    double LPF_coeff = 0.01; // Low pass filter coefficient

    std::chrono::high_resolution_clock::time_point last_time;    // Time point of the last IMU update
    std::chrono::high_resolution_clock::time_point current_time; // Time point of the current IMU update
    double delta_t;                                              // Time elapsed since the last IMU update
};

#endif