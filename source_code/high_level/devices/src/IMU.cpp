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

void IMU::computeRotationMatrix(void)
{
    // Compute the rotation matrix from the body frame to the world frame
    Eigen::Matrix3f Rx, Ry, Rz, R_final;

    Rx << 1, 0, 0,
        0, cosf(euler_deg.angle.roll*DEG2RAD), -sinf(euler_deg.angle.roll*DEG2RAD),
        0, sinf(euler_deg.angle.roll*DEG2RAD), cosf(euler_deg.angle.roll*DEG2RAD);

    Ry << cosf(euler_deg.angle.pitch*DEG2RAD), 0, sinf(euler_deg.angle.pitch*DEG2RAD),
        0, 1, 0,
        -sinf(euler_deg.angle.pitch*DEG2RAD), 0, cosf(euler_deg.angle.pitch*DEG2RAD);

    Rz << cosf(euler_deg.angle.yaw*DEG2RAD), -sinf(euler_deg.angle.yaw*DEG2RAD), 0,
        sinf(euler_deg.angle.yaw*DEG2RAD), cosf(euler_deg.angle.yaw*DEG2RAD), 0,
        0, 0, 1;

    R_final = Rz * Ry * Rx;

    // Convert the Eigen matrix to a Fusion matrix
    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            rotation_matrix.array[i][j] = R_final(i, j);
        }
    }
}

void IMU::processData()
{
    // Get the time elapsed since the last IMU update
    delta_t = getTimeElapsed();

    // Normalize the accelerometer and gyroscope data
    FusionVector accel_fusion = {accel.x / GRAVITY, accel.y / GRAVITY, accel.z / GRAVITY};
    FusionVector gyro_fusion = {gyro.x * RAD2DEG, gyro.y * RAD2DEG, gyro.z * RAD2DEG};

    // Update the AHRS algorithm
    FusionAhrsUpdateNoMagnetometer(&ahrs, gyro_fusion, accel_fusion, delta_t);

    // Get the Euler angles in degrees
    euler_deg = FusionQuaternionToEuler(FusionAhrsGetQuaternion(&ahrs));

    // Compute the angular acceleration
    computeGyroDot();

    // Compute the rotation matrix from the body frame to the world frame
    computeRotationMatrix();
}