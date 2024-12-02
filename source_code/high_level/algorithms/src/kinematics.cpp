#include "kinematics.h"
#include "Eigen/Dense"
#include <cmath>

Direction_Vector Kinematics::computeForwardKinematics(const Joint_Angles &joint_angles, const uint8_t &leg_id)
{
      Direction_Vector foot_pos;

      // Compute foot position usinfg equations from Product of Exponential formulation computed in MATLAB
      foot_pos.x = L4 * sinf(joint_angles.hip_yaw) * sinf(joint_angles.hip_pitch) - L4 * cosf(joint_angles.hip_yaw) * cosf(joint_angles.hip_pitch) * sinf(joint_angles.hip_roll) + L5 * cosf(joint_angles.hip_pitch) * sinf(joint_angles.hip_yaw) * sinf(joint_angles.knee_pitch) + L5 * cosf(joint_angles.knee_pitch) * sinf(joint_angles.hip_yaw) * sinf(joint_angles.hip_pitch) - L5 * cosf(joint_angles.hip_yaw) * cosf(joint_angles.hip_pitch) * cosf(joint_angles.knee_pitch) * sinf(joint_angles.hip_roll) + L5 * cosf(joint_angles.hip_yaw) * sinf(joint_angles.hip_roll) * sinf(joint_angles.hip_pitch) * sinf(joint_angles.knee_pitch);
      foot_pos.y = L4 * cosf(joint_angles.hip_yaw) * sinf(joint_angles.hip_pitch) + L5 * cosf(joint_angles.hip_yaw) * cosf(joint_angles.hip_pitch) * sinf(joint_angles.knee_pitch) + L5 * cosf(joint_angles.hip_yaw) * cosf(joint_angles.knee_pitch) * sinf(joint_angles.hip_pitch) + L4 * cosf(joint_angles.hip_pitch) * sinf(joint_angles.hip_yaw) * sinf(joint_angles.hip_roll) + L5 * cosf(joint_angles.hip_pitch) * cosf(joint_angles.knee_pitch) * sinf(joint_angles.hip_yaw) * sinf(joint_angles.hip_roll) - L5 * sinf(joint_angles.hip_yaw) * sinf(joint_angles.hip_roll) * sinf(joint_angles.hip_pitch) * sinf(joint_angles.knee_pitch);
      foot_pos.z = L5 * cosf(joint_angles.hip_roll) * sinf(joint_angles.hip_pitch) * sinf(joint_angles.knee_pitch) - L4 * cosf(joint_angles.hip_roll) * cosf(joint_angles.hip_pitch) - L5 * cosf(joint_angles.hip_roll) * cosf(joint_angles.hip_pitch) * cosf(joint_angles.knee_pitch) - L2;

      switch (leg_id)
      {
      case LEFT_LEG_ID:
            foot_pos.x -= L1; // Subtract the x distance from the center of the center control module to the center of the hip yaw motor
            break;
      case RIGHT_LEG_ID:
            foot_pos.x += L1; // Add the x distance from the center of the center control module to the center of the hip yaw motor
            break;
      }

      // Return the foot positions
      return foot_pos;
}

Joint_Angles Kinematics::computeInverseKinematics(const Direction_Vector &act_foot_pos, const Direction_Vector &ref_foot_pos, const Joint_Angles &act_joint_angle, const uint8_t &leg_id)
{
      // Initialize foot positions
      Direction_Vector computed_foot_pos;
      Eigen::Vector3f v_ref_foot_pos(ref_foot_pos.x, ref_foot_pos.y, ref_foot_pos.z);
      Eigen::Vector3f v_act_foot_pos(act_foot_pos.x, act_foot_pos.y, act_foot_pos.z);
      Eigen::Vector3f v_foot_pos_error = v_ref_foot_pos - v_act_foot_pos;
      IK_epsilon = v_foot_pos_error.norm(); // Calculation will not be performed if the error is already less than the tolerance

      // Initialize the Jacobian matrix
      Eigen::Matrix3f m_Jacobian;

      // Initialize the joint angles
      Joint_Angles computed_joint_angles = act_joint_angle; // Initial condition for the joint angles is the current joint angles
      Eigen::Vector3f v_delta_joint_angles;

      IK_iteration_count = 0; // Reset the iteration count

      // Newton-Raphson method for inverse kinematics
      while (IK_epsilon > ERROR_TOLERANCE && IK_iteration_count < MAX_ITERATIONS)
      {
            // Initialize the Jacobian matrix
            m_Jacobian(0, 0) = -cosf(computed_joint_angles.hip_yaw) * cosf(computed_joint_angles.hip_roll) * (L5 * cosf(computed_joint_angles.hip_pitch + computed_joint_angles.knee_pitch) + L4 * cosf(computed_joint_angles.hip_pitch));
            m_Jacobian(0, 1) = L4 * cosf(computed_joint_angles.hip_pitch) * sinf(computed_joint_angles.hip_yaw) + L5 * cosf(computed_joint_angles.hip_pitch) * cosf(computed_joint_angles.knee_pitch) * sinf(computed_joint_angles.hip_yaw) + L4 * cosf(computed_joint_angles.hip_yaw) * sinf(computed_joint_angles.hip_roll) * sinf(computed_joint_angles.hip_pitch) - L5 * sinf(computed_joint_angles.hip_yaw) * sinf(computed_joint_angles.hip_pitch) * sinf(computed_joint_angles.knee_pitch) + L5 * cosf(computed_joint_angles.hip_yaw) * cosf(computed_joint_angles.hip_pitch) * sinf(computed_joint_angles.hip_roll) * sinf(computed_joint_angles.knee_pitch) + L5 * cosf(computed_joint_angles.hip_yaw) * cosf(computed_joint_angles.knee_pitch) * sinf(computed_joint_angles.hip_roll) * sinf(computed_joint_angles.hip_pitch);
            m_Jacobian(0, 2) = L5 * cosf(computed_joint_angles.hip_pitch) * cosf(computed_joint_angles.knee_pitch) * sinf(computed_joint_angles.hip_yaw) - L5 * sinf(computed_joint_angles.hip_yaw) * sinf(computed_joint_angles.hip_pitch) * sinf(computed_joint_angles.knee_pitch) + L5 * cosf(computed_joint_angles.hip_yaw) * cosf(computed_joint_angles.hip_pitch) * sinf(computed_joint_angles.hip_roll) * sinf(computed_joint_angles.knee_pitch) + L5 * cosf(computed_joint_angles.hip_yaw) * cosf(computed_joint_angles.knee_pitch) * sinf(computed_joint_angles.hip_roll) * sinf(computed_joint_angles.hip_pitch);
            m_Jacobian(1, 0) = cosf(computed_joint_angles.hip_roll) * sinf(computed_joint_angles.hip_yaw) * (L5 * cosf(computed_joint_angles.hip_pitch + computed_joint_angles.knee_pitch) + L4 * cosf(computed_joint_angles.hip_pitch));
            m_Jacobian(1, 1) = L4 * cosf(computed_joint_angles.hip_yaw) * cosf(computed_joint_angles.hip_pitch) + L5 * cosf(computed_joint_angles.hip_yaw) * cosf(computed_joint_angles.hip_pitch) * cosf(computed_joint_angles.knee_pitch) - L5 * cosf(computed_joint_angles.hip_yaw) * sinf(computed_joint_angles.hip_pitch) * sinf(computed_joint_angles.knee_pitch) - L4 * sinf(computed_joint_angles.hip_yaw) * sinf(computed_joint_angles.hip_roll) * sinf(computed_joint_angles.hip_pitch) - L5 * cosf(computed_joint_angles.hip_pitch) * sinf(computed_joint_angles.hip_yaw) * sinf(computed_joint_angles.hip_roll) * sinf(computed_joint_angles.knee_pitch) - L5 * cosf(computed_joint_angles.knee_pitch) * sinf(computed_joint_angles.hip_yaw) * sinf(computed_joint_angles.hip_roll) * sinf(computed_joint_angles.hip_pitch);
            m_Jacobian(1, 2) = L5 * cosf(computed_joint_angles.hip_yaw) * cosf(computed_joint_angles.hip_pitch) * cosf(computed_joint_angles.knee_pitch) - L5 * cosf(computed_joint_angles.hip_yaw) * sinf(computed_joint_angles.hip_pitch) * sinf(computed_joint_angles.knee_pitch) - L5 * cosf(computed_joint_angles.hip_pitch) * sinf(computed_joint_angles.hip_yaw) * sinf(computed_joint_angles.hip_roll) * sinf(computed_joint_angles.knee_pitch) - L5 * cosf(computed_joint_angles.knee_pitch) * sinf(computed_joint_angles.hip_yaw) * sinf(computed_joint_angles.hip_roll) * sinf(computed_joint_angles.hip_pitch);
            m_Jacobian(2, 0) = sinf(computed_joint_angles.hip_roll) * (L5 * cosf(computed_joint_angles.hip_pitch + computed_joint_angles.knee_pitch) + L4 * cosf(computed_joint_angles.hip_pitch));
            m_Jacobian(2, 1) = cosf(computed_joint_angles.hip_roll) * (L5 * sinf(computed_joint_angles.hip_pitch + computed_joint_angles.knee_pitch) + L4 * sinf(computed_joint_angles.hip_pitch));
            m_Jacobian(2, 2) = L5 * sinf(computed_joint_angles.hip_pitch + computed_joint_angles.knee_pitch) * cosf(computed_joint_angles.hip_roll);

            // SR Inverse usinfg Levenberg-Marquardt method solved usinfg ldlt decomposition as in this Ax = b, A is symmetric and positive definite
            v_delta_joint_angles = (m_Jacobian.transpose() * m_Jacobian + IK_lambda * Eigen::Matrix3f::Identity()).ldlt().solve(m_Jacobian.transpose() * v_foot_pos_error);

            // Update the joint angles
            computed_joint_angles.hip_roll += v_delta_joint_angles(0);
            computed_joint_angles.hip_pitch += v_delta_joint_angles(1);
            computed_joint_angles.knee_pitch += v_delta_joint_angles(2);

            // Recompute the foot position
            computed_foot_pos = computeForwardKinematics(computed_joint_angles, leg_id);
            v_act_foot_pos = {computed_foot_pos.x, computed_foot_pos.y, computed_foot_pos.z};
            v_foot_pos_error = v_ref_foot_pos - v_act_foot_pos;

            IK_epsilon = v_foot_pos_error.norm(); // Compute the error norm to check for convergence

            IK_iteration_count++; // Increment the iteration count
      }

      computed_joint_angles.hip_yaw = 0; // Hip yaw is always zero

      return computed_joint_angles;
}
