#include "IMU.h"
#include "main.h"
#include "cmsis_os.h"
#include "gpio.h"
#include "tim.h"
#include "User_Math.h"

IMU::IMU() {
    ax = ay = az = 0;
    gx = gy = gz = 0;
    yaw_rad = pitch_rad = roll_rad = 0;
    yaw_deg = pitch_deg = roll_deg = 0;
    temp = 0;
    FusionAhrsInitialise(&IMU_AHRS);
}

void IMU::heatControl() {
    if (temp < DESIRED_TEMP) {
        htim3.Instance->CCR4 = MAX_OUTPUT;
    } else {
        htim3.Instance->CCR4 = 0;
    }
}

void IMU::updateRaw(const float accel[3], const float gyro[3], float temperature) {
    ax = accel[0];
    ay = accel[1];
    az = accel[2];

    gx = gyro[0];
    gy = gyro[1];
    gz = gyro[2];

    temp = temperature;
}

void IMU::processData() {
    // Normalize accelerometer and convert gyroscope to degrees per second
    FusionVector Accel = { ax / GRAVITY, ay / GRAVITY, az / GRAVITY };
    FusionVector Gyro = { gx * RAD2DEG, gy * RAD2DEG, gz * RAD2DEG };

    // Update the AHRS system (no magnetometer)
    FusionAhrsUpdateNoMagnetometer(&IMU_AHRS, Gyro, Accel, 0.001f);

    // Get the Euler angles from the AHRS quaternion
    FusionEuler IMU_Euler = FusionQuaternionToEuler(FusionAhrsGetQuaternion(&IMU_AHRS));

    // Update the Euler angles in both degrees and radians
    yaw_deg = IMU_Euler.angle.yaw;
    pitch_deg = IMU_Euler.angle.pitch;
    roll_deg = IMU_Euler.angle.roll;

    yaw_rad = yaw_deg * DEG2RAD;
    pitch_rad = pitch_deg * DEG2RAD;
    roll_rad = roll_deg * DEG2RAD;
}
