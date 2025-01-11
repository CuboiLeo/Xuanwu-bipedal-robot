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
    gyro_dot.x = (1 - LPF_coeff) * gyro_dot.x + LPF_coeff * (gyro.x - prev_gyro.x) / delta_t;
    gyro_dot.y = (1 - LPF_coeff) * gyro_dot.y + LPF_coeff * (gyro.y - prev_gyro.y) / delta_t;
    gyro_dot.z = (1 - LPF_coeff) * gyro_dot.z + LPF_coeff * (gyro.z - prev_gyro.z) / delta_t;

    prev_gyro = gyro; // Update the previous angular velocity
}

void IMU::processData()
{
    // Get the time elapsed since the last IMU update
    delta_t = getTimeElapsed();

    // Normalize the accelerometer and gyroscope data
    FusionVector accel_fusion = {(float)accel.x / GRAVITY, (float)accel.y / GRAVITY, (float)accel.z / GRAVITY};
    FusionVector gyro_fusion = {(float)gyro.x * RAD2DEG, (float)gyro.y * RAD2DEG, (float)gyro.z * RAD2DEG};

    // Update the AHRS algorithm
    FusionAhrsUpdateNoMagnetometer(&ahrs, gyro_fusion, accel_fusion, delta_t);

    // Get the Euler angles in degrees
    FusionEuler euler_deg = FusionQuaternionToEuler(FusionAhrsGetQuaternion(&ahrs));
    euler_angles = {euler_deg.angle.roll * DEG2RAD, euler_deg.angle.pitch * DEG2RAD, euler_deg.angle.yaw * DEG2RAD};

    // Compute the angular acceleration
    computeGyroDot();

    // Compute the rotation matrix from the body frame to the world frame
    rotation_matrix = eul_to_rotm(euler_angles);
}