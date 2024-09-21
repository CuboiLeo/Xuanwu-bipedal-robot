#ifndef IMU_H
#define IMU_H

#ifdef __cplusplus
extern "C" {
#endif

#include "Fusion.h"
#include "BMI088driver.h"
#include "BMI088Middleware.h"

class IMU {
public:
    IMU();  // Constructor to initialize the IMU class
    void updateRaw(const float accel[3], const float gyro[3], float temp);  // Method to update sensor values
    void processData();  // Method to process IMU data and calculate Euler angles
	void heatControl();  // Method to control the heating element based on temperature
    // Getter methods for IMU values
    float getAx() const { return ax; }
    float getAy() const { return ay; }
    float getAz() const { return az; }

    float getGx() const { return gx; }
    float getGy() const { return gy; }
    float getGz() const { return gz; }

    float getYawRad() const { return yaw_rad; }
    float getPitchRad() const { return pitch_rad; }
    float getRollRad() const { return roll_rad; }

    float getYawDeg() const { return yaw_deg; }
    float getPitchDeg() const { return pitch_deg; }
    float getRollDeg() const { return roll_deg; }

    float getTemp() const { return temp; }

private:
	float static constexpr DESIRED_TEMP = 40.0f;
	float static constexpr MAX_OUTPUT = 500;
    float ax, ay, az;  // Accelerometer values in m/s^2
    float gx, gy, gz;  // Gyroscope values in rad/s
    float yaw_rad, pitch_rad, roll_rad;  // Euler angles in radians
    float yaw_deg, pitch_deg, roll_deg;  // Euler angles in degrees
    float temp;  // Temperature in degrees Celsius

    FusionAhrs IMU_AHRS;  // Fusion AHRS object for quaternion calculations
};

#ifdef __cplusplus
}
#endif

#endif // IMU_H