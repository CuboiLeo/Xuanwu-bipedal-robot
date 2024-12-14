#ifndef DYNAMICS_H
#define DYNAMICS_H

#include "stdint.h"
#include "robot_types.h"
#include "robot_configs.h"
#include "Fusion.h"
#include "Eigen/Dense"

class Dynamics
{
public:
    Direction_Vector computeCoMAccel(const Direction_Vector &CoM_pos, const Direction_Vector &accel, const Direction_Vector &gyro, const Direction_Vector &gyro_dot);
    Direction_Vector computeZMPPos(const Direction_Vector &CoM, const Direction_Vector &CoM_accel);
};

#endif // DYNAMICS_H
