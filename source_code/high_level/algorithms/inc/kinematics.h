#ifndef KINEMATICS_H
#define KINEMATICS_H

#include <stdint.h>
#include "robot_types.h"
#include "robot_configs.h"
#include "user_math.h"

class Kinematics
{
public:
      static constexpr uint8_t MAX_ITERATIONS = 50;    // Maximum number of iterations for the inverse kinematics
      static constexpr float ERROR_TOLERANCE = 0.005f; // Error tolerance for the inverse kinematics

      Direction_Vector computeForwardKinematics(const Joint_Angles &joint_angles, const uint8_t &leg_id);
      Joint_Angles computeInverseKinematics(const Direction_Vector &act_foot_pos, const Direction_Vector &ref_foot_pos, const Joint_Angles &act_angle, const uint8_t &leg_id);

private:
      uint8_t IK_iteration_count = 0; // Iteration count for the inverse kinematics
      float IK_epsilon = 1.0f;        // Error for the inverse kinematics
      float IK_lambda = 0.001f;       // Damping factor for the inverse kinematics - SR Inverse Levenberg-Marquardt method
};

#endif
