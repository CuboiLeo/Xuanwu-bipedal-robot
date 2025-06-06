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
		prev_temp_error = 0;
		temp_error = 0;
    temp = 0;
    rotation_matrix = {1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f};
    FusionAhrsInitialise(&IMU_AHRS);
}

void IMU::heatControl()
{
		prev_temp_error = temp_error;
		temp_error = DESIRED_TEMP - temp;
    htim3.Instance->CCR4 = CLIP(temp_error*KP + (temp_error - prev_temp_error)*KD, 0, MAX_OUTPUT);
}

void IMU::updateRaw(const float accel[3], const float gyro[3], float temperature)
{
    this->accel = {accel[0],accel[1],accel[2]};
    this->gyro = {gyro[0]-GYRO_X_OFFSET, gyro[1]-GYRO_Y_OFFSET, gyro[2]-GYRO_Z_OFFSET};
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
