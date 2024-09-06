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
	float yaw; //rad
	float pitch; //rad
	float roll; //rad
}IMU_t;

IMU_t g_IMU;
#endif