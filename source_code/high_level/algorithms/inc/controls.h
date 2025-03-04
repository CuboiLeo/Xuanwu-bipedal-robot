#ifndef CONTROLS_H
#define CONTROLS_H

#include "robot_types.h"
#include "Eigen/Dense"
#include "Fusion.h"
#include "robot_configs.h"

class Controls
{
public:
    Joint_Angles_Two_Legs controlZMPPos(const Position &ref_ZMP_pos, const Position &act_ZMP_pos, const Joint_Angles_Two_Legs &act_joint_angles);

private:
    inline static constexpr double ZMP_error_gain = 0.3;
    inline static constexpr double ZMP_error_threshold_x = 0.05; // Support polygon in x direction (m)
    inline static constexpr double ZMP_error_threshold_y = 0.001; // Support polygon in y direction (m)
    inline static constexpr float LAMBDA = 0.001f;               // Damping factor for the inverse kinematics - SR Inverse Levenberg-Marquardt method
};

#endif
