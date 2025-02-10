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
    Acceleration computeCoMAccel(const Position &CoM_pos, const Eigen::Vector3d &accel, const Eigen::Vector3d &gyro, const Eigen::Vector3d &gyro_dot);
    Position computeZMPPos(const Position &CoM, const Acceleration &CoM_accel);
};

#endif // DYNAMICS_H
