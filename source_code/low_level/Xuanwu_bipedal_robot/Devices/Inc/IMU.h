#ifndef IMU_H
#define IMU_H

#include "Fusion.h"
#include "BMI088driver.h"
#include "BMI088Middleware.h"
#include "User_Math.h"

static constexpr float IMU_TASK_PERIOD = 0.001f; // IMU task period in seconds

class IMU
{
public:
    IMU();                                                                 // Constructor to initialize the IMU class
    void updateRaw(const float accel[3], const float gyro[3], float temp); // Method to update sensor values
    void processData();                                                    // Method to process IMU data and calculate Euler angles
    void heatControl();                                                    // Method to control the heating element based on temperature
    // Getter methods for IMU values
    FusionVector getAccel() const { return accel; }
    FusionVector getGyro() const { return gyro; }
    FusionEuler getEulerRad() const { return euler_rad; }
    FusionEuler getEulerDeg() const { return euler_deg; }
    FusionMatrix getRotationMatrix() const { return rotation_matrix; }
    float getTemp() const { return temp; }

private:
    float static constexpr DESIRED_TEMP = 50.0f;
    float static constexpr MAX_OUTPUT = 500.0f;
    float static constexpr FILTER_COEFFICIENT = 0.01f;
		float static constexpr KP = 50.0f;
		float static constexpr KD = 10.0f;
		float static constexpr GYRO_X_OFFSET = 0.002919685f;
		float static constexpr GYRO_Y_OFFSET = 0.002117431f;
		float static constexpr GYRO_Z_OFFSET = 0.000218696f;
    FusionVector accel;           // Accelerometer values in m/s^2
    FusionVector gyro;            // Gyroscope values in rad/s
    FusionEuler euler_rad;        // Euler angles in radians
    FusionEuler euler_deg;        // Euler angles in degrees
		float temp_error;
		float prev_temp_error;
    float temp;                   // Temperature in degrees Celsius
    FusionMatrix rotation_matrix; // Rotation matrix for IMU orientation
    FusionAhrs IMU_AHRS;          // Fusion AHRS object for quaternion calculations
};

#endif // IMU_H
