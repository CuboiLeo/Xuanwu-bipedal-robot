#ifndef STM32_PROTOCOL_H
#define STM32_PROTOCOL_H

#include "CAN.h"
#include "user_math.h"
#include "motor.h"
#include "IMU.h"
#include "command.h"

#define PACKAGE_SIZE (8)
#define SEND_PACKAGE_NUM (6)
#define RECEIVE_PACKAGE_NUM (7)
#define CAN_SEND_START_ID (0x21)
#define CAN_RECEIVE_START_ID (0x01)

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
#define G_MIN (-2000.0f)
#define G_MAX (2000.0f)

// Define the range of the data for command
#define LINEAR_VEL_MIN (-1.0f)
#define LINEAR_VEL_MAX (1.0f)
#define ANGULAR_VEL_MIN (-0.5f)
#define ANGULAR_VEL_MAX (0.5f)

class STM32
{
public:
    void sendData(CAN &can, Motor &motor);
    void receiveData(CAN &can, Motor &motor, IMU &imu, Command &command);

private:
    uint8_t send_data[SEND_PACKAGE_NUM][PACKAGE_SIZE] = {};
    uint8_t receive_data[RECEIVE_PACKAGE_NUM][PACKAGE_SIZE] = {};
};

#endif