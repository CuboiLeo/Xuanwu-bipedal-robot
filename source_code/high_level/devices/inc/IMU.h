#ifndef IMU_H
#define IMU_H

#include <stdint.h>
#include "robot_types.h"

class IMU
{
public:
    direction_vector getAccel() const { return accel; };
    direction_vector getGyro() const { return gyro; };

    void setAccel(const direction_vector accel) { this->accel = accel; };
    void setGyro(const direction_vector gyro) { this->gyro = gyro; };

private:
    direction_vector accel;
    direction_vector gyro;
};

#endif