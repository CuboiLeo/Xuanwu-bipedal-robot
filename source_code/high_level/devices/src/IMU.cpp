#include "IMU.h"

IMU::IMU()
{
    // Initialize the AHRS algorithm
    FusionAhrsInitialise(&ahrs);
    last_time = std::chrono::high_resolution_clock::now();
}

double IMU::getTimeElapsed()
{
    // Get the time elapsed since the last IMU update
    current_time = std::chrono::high_resolution_clock::now();
    std::chrono::duration<float> time_elapsed = current_time - last_time;

    last_time = current_time;

    // Convert the time elapsed to seconds
    return time_elapsed.count();
}

void IMU::computeGyroDot(void)
{
    // Compute the angular acceleration using a low pass filter to reduce noise
    gyro_dot = (1 - LPF_coeff) * gyro_dot + LPF_coeff * (gyro - prev_gyro) / delta_t;
    prev_gyro = gyro; // Update the previous angular velocity
}

void IMU::computeVel(void)
{
    // Integrate the acceleration to get the velocity
    vel = vel +  accel * delta_t;
}

void IMU::processData()
{
    // Get the time elapsed since the last IMU update
    delta_t = getTimeElapsed();
    delta_t = std::min(delta_t, 0.01); // Ensure that delta_t is no bigger than a threashold when program is paused

    // Normalize the accelerometer and gyroscope data
    FusionVector accel_fusion = {(float)raw_accel.x / GRAVITY, (float)raw_accel.y / GRAVITY, (float)raw_accel.z / GRAVITY};
    FusionVector gyro_fusion = {(float)raw_gyro.x * RAD2DEG, (float)raw_gyro.y * RAD2DEG, (float)raw_gyro.z * RAD2DEG};

    // Update the AHRS algorithm
    FusionAhrsUpdateNoMagnetometer(&ahrs, gyro_fusion, accel_fusion, delta_t);

    // Get the Euler angles in degrees
    FusionEuler euler_deg = FusionQuaternionToEuler(FusionAhrsGetQuaternion(&ahrs));
    euler_angles = {euler_deg.angle.roll * DEG2RAD, euler_deg.angle.pitch * DEG2RAD, euler_deg.angle.yaw * DEG2RAD};

    // Compute the rotation matrix from the body frame to the world frame
    rotation_matrix = eul_to_rotm(euler_angles);

    // Compute the acceleration in the world frame
    Eigen::Vector3d v_raw_accel = {raw_accel.x, raw_accel.y, raw_accel.z};
    accel = rotation_matrix * v_raw_accel;

    // Compute the angular velocity in the world frame
    Eigen::Vector3d v_raw_gyro = {raw_gyro.x, raw_gyro.y, raw_gyro.z};
    gyro = rotation_matrix * v_raw_gyro;

    // Compute the angular acceleration
    computeGyroDot();
}