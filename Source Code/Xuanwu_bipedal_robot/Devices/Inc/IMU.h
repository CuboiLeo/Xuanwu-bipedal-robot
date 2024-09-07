#ifndef IMU_H
#define IMU_H

typedef struct IMU
{
	float ax; // m/s^2
	float ay; // m/s^2
	float az; // m/s^2
	float gx;	// rad/s
	float gy;	//rad/s
	float gz; // rad/s
	float yaw_rad; //rad
	float pitch_rad; //rad
	float roll_rad; //rad
	float yaw_deg; //deg
	float pitch_deg; //deg
	float roll_deg; //deg

	float temp; // degC
}IMU_t;

extern IMU_t g_IMU;
#endif