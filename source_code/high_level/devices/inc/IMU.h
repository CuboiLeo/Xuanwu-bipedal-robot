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
    Direction_Vector getAccel() const { return accel; };
    Direction_Vector getGyro() const { return gyro; };
    Direction_Vector getGyroDot() const { return gyro_dot; };
    FusionEuler getEuler() const { return euler_deg; };
    FusionMatrix getRotationMatrix() const { return rotation_matrix; };
    

    void setAccel(const Direction_Vector accel) { this->accel = accel; };
    void setGyro(const Direction_Vector gyro) { this->gyro = gyro; };
    void computeGyroDot(void);
    void computeRotationMatrix(void);
    double getTimeElapsed(void);
    void processData(void);

private:
    Direction_Vector accel = {};     // Acceleration in m/s^2
    Direction_Vector gyro = {};      // Angular velocity in rad/s
    Direction_Vector prev_gyro = {}; // Previous angular velocity in rad/s
    Direction_Vector gyro_dot = {};  // Angular acceleration in rad/s^2

    FusionAhrs ahrs;                   // AHRS algorithm structure
    FusionEuler euler_deg = {};       // Euler angles in degrees for roll, pitch, and yaw
    FusionMatrix rotation_matrix = {}; // Rotation matrix from body frame to world frame

    float LPF_coeff = 0.01f; // Low pass filter coefficient

    std::chrono::high_resolution_clock::time_point last_time;    // Time point of the last IMU update
    std::chrono::high_resolution_clock::time_point current_time; // Time point of the current IMU update
    double delta_t;                  // Time elapsed since the last IMU update
};

#endif