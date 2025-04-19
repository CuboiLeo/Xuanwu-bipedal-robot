#ifndef STM32_PROTOCOL_H
#define STM32_PROTOCOL_H

#include "CAN.h"
#include "user_math.h"
#include "motor.h"
#include "IMU.h"
#include "command.h"

#define PACKAGE_SIZE (8)
#define SEND_PACKAGE_NUM (7)
#define SEND_PACKAGE_NUM_LITE (3)
#define RECEIVE_PACKAGE_NUM (9)
#define RECEIVE_PACKAGE_NUM_LITE (7)
#define CAN_SEND_START_ID (0x21)
#define CAN_RECEIVE_START_ID (0x01)
#define USE_LITE_PACKAGE (true)

// Define the range of the data for motor control
#define P_MIN (-12.5f)
#define P_MAX (12.5f)
#define V_MIN (-30.0f)
#define V_MAX (30.0f)
#define T_MIN (-10.0f)
#define T_MAX (10.0f)

// Define the range of the data for IMU
#define A_MIN (-3*GRAVITY)
#define A_MAX (3*GRAVITY)
#define G_MIN (-2000.0f*DEG2RAD)
#define G_MAX (2000.0f*DEG2RAD)

// Define the range of the data for command
#define LINEAR_VEL_MIN (-0.5f)
#define LINEAR_VEL_MAX (0.5f)
#define ANGULAR_VEL_MIN (-0.5f)
#define ANGULAR_VEL_MAX (0.5f)

class STM32
{
public:
    void sendData(CAN &can);
    void receiveData(CAN &can);
    void encodeData(Motor &motor);
    void encodeDataLite(Motor &motor);
    void decodeData(Motor &motor, IMU &imu, Command &command);
    void decodeDataLite(Motor &motor, IMU &imu, Command &command);

private:
    uint8_t send_data[SEND_PACKAGE_NUM][PACKAGE_SIZE] = {};
    uint8_t receive_data[RECEIVE_PACKAGE_NUM][PACKAGE_SIZE] = {};
    uint8_t send_data_lite[SEND_PACKAGE_NUM_LITE][PACKAGE_SIZE] = {};
    uint8_t receive_data_lite[RECEIVE_PACKAGE_NUM_LITE][PACKAGE_SIZE] = {};
};

#endif