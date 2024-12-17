#ifndef CONTROLS_H
#define CONTROLS_H

#include "robot_types.h"
#include "Eigen/Dense"
#include "Fusion.h"
#include "robot_configs.h"

class Controls
{
public:
    Joint_Angles_Two_Legs controlZMPPos(const Direction_Vector &act_ZMP_pos, const Direction_Vector &ref_ZMP_pos, const Joint_Angles_Two_Legs &act_joint_angles, const FusionMatrix &rotation_matrix);

private:
    inline static constexpr float ZMP_ERROR_GAIN = 0.05f; // Gain for the ZMP error
    inline static constexpr float LAMBDA = 0.001f;       // Damping factor for the inverse kinematics - SR Inverse Levenberg-Marquardt method
};

#endif
