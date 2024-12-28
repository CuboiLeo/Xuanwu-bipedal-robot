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
#define RECEIVE_PACKAGE_NUM (7)
#define CAN_SEND_START_ID (0x01)
#define CAN_RECEIVE_START_ID (0x21)

class Orin
{
public:
    void sendData(const Motor &motor, const IMU &imu, const Direction_Vector &ref_vel, const Direction_Vector &ref_angular_vel);
    void receiveData(void);
    void decodeData(Motor &motor);

private:
    uint8_t send_data[SEND_PACKAGE_NUM][PACKAGE_SIZE] = {};
    uint8_t receive_data[RECEIVE_PACKAGE_NUM][PACKAGE_SIZE] = {};
    uint8_t watchdog = 0;
};

#endif
