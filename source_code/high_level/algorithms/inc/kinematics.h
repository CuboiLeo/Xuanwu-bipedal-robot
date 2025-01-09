#ifndef KINEMATICS_H
#define KINEMATICS_H

#include <stdint.h>
#include "robot_types.h"
#include "robot_configs.h"
#include "user_math.h"
#include "Fusion.h"

class Kinematics
{
public:
      Direction_Vector computeCoMFK(const Joint_Angles_Two_Legs &joint_angles, const FusionMatrix &rotation_matrix);
      Joint_Angles_Two_Legs computeCoMIK(const Direction_Vector &act_CoM_pos, const Direction_Vector &ref_CoM_pos, const Joint_Angles_Two_Legs &act_joint_angles, const FusionMatrix &rotation_matrix);
      Direction_Vector computeFootFK(const Joint_Angles &joint_angles, const uint8_t &leg_id);
      Joint_Angles computeFootIK(const Direction_Vector &act_foot_pos, const Direction_Vector &ref_foot_pos, const Joint_Angles &act_joint_angles, const uint8_t &leg_id);

private:
      int IK_iteration_count = 0; // Iteration count for the inverse kinematics
      inline static constexpr int MAX_ITERATIONS = 50;    // Maximum number of iterations for the inverse kinematics
      inline static constexpr float ERROR_TOLERANCE = 0.0005f; // Error tolerance for the inverse kinematics
      inline static constexpr float IK_LAMBDA = 0.001f;       // Damping factor for the inverse kinematics - SR Inverse Levenberg-Marquardt method
};

#endif
