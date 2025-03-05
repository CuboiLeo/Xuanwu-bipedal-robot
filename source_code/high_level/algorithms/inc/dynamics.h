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
    Wrench computeFootWrench(const Joint_Torques &torques, const Joint_Angles &angles, const uint8_t &leg_id);
    Joint_Torques computeGRFTorques(const Wrench &GRF, const Joint_Angles &angles, const uint8_t &leg_id);

private:
    Position ZMP_pos = {0.0, 0.0, 0.0};
    double filter_coeff = 0.01;
};

#endif // DYNAMICS_H
