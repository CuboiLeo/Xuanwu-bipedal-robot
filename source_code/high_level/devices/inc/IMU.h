#ifndef IMU_H
#define IMU_H

#include <stdint.h>
#include "robot_types.h"

class IMU
{
public:
    Direction_Vector getAccel() const { return accel; };
    Direction_Vector getGyro() const { return gyro; };

    void setAccel(const Direction_Vector accel) { this->accel = accel; };
    void setGyro(const Direction_Vector gyro) { this->gyro = gyro; };

private:
    Direction_Vector accel;
    Direction_Vector gyro;
};

#endif