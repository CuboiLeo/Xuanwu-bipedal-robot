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
      Position computeCoMPos(const Joint_Angles_Two_Legs &joint_angles, const Eigen::Matrix3d &rotation_matrix);
      Eigen::Matrix4d computeFootFK(const Joint_Angles &joint_angles, const uint8_t &leg_id);
      Joint_Angles computeFootIK(const Pose &act_foot_pos, const Pose &ref_foot_pos, const uint8_t &leg_id);

private:
      int iteration_count = 0;                           // Iteration count for the inverse kinematics
      inline static constexpr int MAX_ITERATIONS = 50;   // Maximum number of iterations for the inverse kinematics
      inline static constexpr float EPSILON_W = 0.001f; // Error tolerance for for angular velocity w
      inline static constexpr float EPSILON_V = 0.001f; // Error tolerance for for linear velocity v
      inline static constexpr float LAMBDA = 0.001f;     // Damping factor for the inverse kinematics - SR Inverse Levenberg-Marquardt method
};

#endif
