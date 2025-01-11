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
    Position computeCoMAccel(const Position &CoM_pos, const Acceleration &accel, const Angular_Velocity &gyro, const Angular_Acceleration &gyro_dot);
    Position computeZMPPos(const Position &CoM, const Acceleration &CoM_accel);
};

#endif // DYNAMICS_H
