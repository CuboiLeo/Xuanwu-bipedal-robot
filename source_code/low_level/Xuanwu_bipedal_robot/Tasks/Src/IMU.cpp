#include "IMU.h"
#include "main.h"
#include "cmsis_os.h"
#include "gpio.h"
#include "tim.h"
#include "User_Math.h"

IMU::IMU()
{
    accel = {0.0f, 0.0f, 0.0f};
    gyro = {0.0f, 0.0f, 0.0f};
    euler_rad = {0.0f, 0.0f, 0.0f};
    euler_deg = {0.0f, 0.0f, 0.0f};
    temp = 0;
    rotation_matrix = {1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f};
    FusionAhrsInitialise(&IMU_AHRS);
}

void IMU::heatControl()
{
    if (temp < DESIRED_TEMP)
    {
        htim3.Instance->CCR4 = MAX_OUTPUT;
    }
    else
    {
        htim3.Instance->CCR4 = 0;
    }
}

void IMU::updateRaw(const float accel[3], const float gyro[3], float temperature)
{
    this->accel = {accel[0],accel[1],accel[2]};
    this->gyro = {gyro[0], gyro[1], gyro[2]};
    temp = temperature;
}

void IMU::processData()
{
    // Normalize accelerometer and convert gyroscope to degrees per second
    FusionVector Accel = {accel.axis.x / GRAVITY, accel.axis.y / GRAVITY, accel.axis.z / GRAVITY};
    FusionVector Gyro = {gyro.axis.x * RAD2DEG, gyro.axis.y * RAD2DEG, gyro.axis.z * RAD2DEG};

    // Update the AHRS system (no magnetometer)
    FusionAhrsUpdateNoMagnetometer(&IMU_AHRS, Gyro, Accel, IMU_TASK_PERIOD);

    // Get the Euler angles from the AHRS quaternion
    euler_deg = FusionQuaternionToEuler(FusionAhrsGetQuaternion(&IMU_AHRS));
    // convert to radians
    euler_rad = {euler_deg.angle.roll * DEG2RAD, euler_deg.angle.pitch * DEG2RAD, euler_deg.angle.yaw * DEG2RAD};

    // Update the rotation matrix
    rotation_matrix = FusionQuaternionToMatrix(FusionAhrsGetQuaternion(&IMU_AHRS));
}
