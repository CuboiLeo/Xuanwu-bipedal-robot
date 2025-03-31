#ifndef ORIN_NX_H
#define ORIN_NX_H

#include "CAN_BSP.h"
#include "stdint.h"
#include "DM4310driver.h"
#include "Motor.h"
#include "IMU.h"
#include "Robot_Types.h"
#include "Robot.h"

#define PACKAGE_SIZE (8)
#define SEND_PACKAGE_NUM (9)
#define SEND_PACKAGE_NUM_LITE (7)
#define RECEIVE_PACKAGE_NUM (7)
#define RECEIVE_PACKAGE_NUM_LITE (3)
#define CAN_SEND_START_ID (0x01)
#define CAN_RECEIVE_START_ID (0x21)
#define USE_LITE_PACKAGE (true)

class Orin
{
public:
    void sendData(const Motor &motor, const IMU &imu, const Direction_Vector &ref_vel, const Direction_Vector &ref_angular_vel);
    void sendDataLite(const Motor &motor, const IMU &imu, const Direction_Vector &ref_vel, const Direction_Vector &ref_angular_vel);
    void receiveData(void);
    void receiveDataLite(void);
    void decodeData(Motor &motor);
    void decodeDataLite(Motor &motor);

private:
    uint8_t send_data[SEND_PACKAGE_NUM][PACKAGE_SIZE] = {};
    uint8_t receive_data[RECEIVE_PACKAGE_NUM][PACKAGE_SIZE] = {};
    uint8_t watchdog = 0;
    float filter_coeff = 0.01f;

    uint8_t send_data_lite[SEND_PACKAGE_NUM_LITE][PACKAGE_SIZE] = {};
    uint8_t receive_data_lite[RECEIVE_PACKAGE_NUM_LITE][PACKAGE_SIZE] = {};

    float kps[10] = {40.0f, 40.0f, 40.0f, 40.0f, 10.0f, 40.0f, 40.0f, 40.0f, 40.0f, 10.0f};
    float kds[10] = {0.25f, 0.25f, 0.25f, 0.25f, 0.15f, 0.25f, 0.25f, 0.25f, 0.25f, 0.15f};
    float tor_limit = 5.0f;
};

#endif
