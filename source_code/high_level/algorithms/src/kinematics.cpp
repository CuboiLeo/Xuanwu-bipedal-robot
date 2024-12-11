#include "kinematics.h"
#include "Eigen/Dense"
#include <cmath>

Direction_Vector Kinematics::computeFootFK(const Joint_Angles &joint_angles, const uint8_t &leg_id)
{
      Direction_Vector foot_pos;

      // Compute foot position in center's frame using Product of Exponential formulation computed in MATLAB
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

Joint_Angles Kinematics::computeFootIK(const Direction_Vector &act_foot_pos, const Direction_Vector &ref_foot_pos, const Joint_Angles &act_joint_angles, const uint8_t &leg_id)
{
      // Initialize foot positions
      Direction_Vector computed_foot_pos;
      Eigen::Vector3f v_ref_foot_pos(ref_foot_pos.x, ref_foot_pos.y, ref_foot_pos.z);
      Eigen::Vector3f v_act_foot_pos(act_foot_pos.x, act_foot_pos.y, act_foot_pos.z);
      Eigen::Vector3f v_foot_pos_error = v_ref_foot_pos - v_act_foot_pos;
      float IK_epsilon = v_foot_pos_error.norm(); // Calculation will not be performed if the error is already less than the tolerance

      // Initialize the Jacobian matrix
      Eigen::Matrix3f m_Jacobian;

      // Initialize the joint angles
      Joint_Angles computed_joint_angles = act_joint_angles; // Initial condition for the joint angles is the current joint angles
      computed_joint_angles.hip_pitch = 0.3f;                // Initial condition for the hip pitch so that knee always bends in the forward direction
      Eigen::Vector3f v_delta_joint_angles;

      uint8_t IK_iteration_count = 0; // Reset the iteration count

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

            // SR Inverse usinfg Levenberg-Marquardt method solved using ldlt decomposition as in this Ax = b, A is symmetric and positive definite
            v_delta_joint_angles = (m_Jacobian.transpose() * m_Jacobian + IK_LAMBDA * Eigen::Matrix3f::Identity()).ldlt().solve(m_Jacobian.transpose() * v_foot_pos_error);

            // Update the joint angles
            computed_joint_angles.hip_roll += v_delta_joint_angles(0);
            computed_joint_angles.hip_pitch += v_delta_joint_angles(1);
            computed_joint_angles.knee_pitch += v_delta_joint_angles(2);

            // Recompute the foot position
            computed_foot_pos = computeFootFK(computed_joint_angles, leg_id);
            v_act_foot_pos = {computed_foot_pos.x, computed_foot_pos.y, computed_foot_pos.z};
            v_foot_pos_error = v_ref_foot_pos - v_act_foot_pos;

            IK_epsilon = v_foot_pos_error.norm(); // Compute the error norm to check for convergence

            IK_iteration_count++; // Increment the iteration count
      }

      computed_joint_angles.hip_yaw = 0; // Hip yaw is always zero

      return computed_joint_angles;
}

Direction_Vector Kinematics::computeCenterFK(const Joint_Angles &joint_angles, const uint8_t &leg_id)
{
      Direction_Vector center_pos;

      // Compute center position in center's frame using Product of Exponential formulation computed in MATLAB
      switch (leg_id)
      {
      case LEFT_LEG_ID:
            center_pos.x = L1 * cosf(joint_angles.hip_yaw) * cosf(joint_angles.hip_roll) - L2 * sinf(joint_angles.hip_roll) - L1;
            center_pos.y = L4 * sinf(joint_angles.knee_pitch) + L1 * cosf(joint_angles.hip_pitch) * cosf(joint_angles.knee_pitch) * sinf(joint_angles.hip_yaw) + L2 * cosf(joint_angles.hip_roll) * cosf(joint_angles.hip_pitch) * sinf(joint_angles.knee_pitch) + L2 * cosf(joint_angles.hip_roll) * cosf(joint_angles.knee_pitch) * sinf(joint_angles.hip_pitch) - L1 * sinf(joint_angles.hip_yaw) * sinf(joint_angles.hip_pitch) * sinf(joint_angles.knee_pitch) + L1 * cosf(joint_angles.hip_yaw) * cosf(joint_angles.hip_pitch) * sinf(joint_angles.hip_roll) * sinf(joint_angles.knee_pitch) + L1 * cosf(joint_angles.hip_yaw) * cosf(joint_angles.knee_pitch) * sinf(joint_angles.hip_roll) * sinf(joint_angles.hip_pitch);
            center_pos.z = L4 * cosf(joint_angles.knee_pitch) - L4 - L2 + L2 * cosf(joint_angles.hip_roll) * cosf(joint_angles.hip_pitch) * cosf(joint_angles.knee_pitch) - L1 * cosf(joint_angles.hip_pitch) * sinf(joint_angles.hip_yaw) * sinf(joint_angles.knee_pitch) - L1 * cosf(joint_angles.knee_pitch) * sinf(joint_angles.hip_yaw) * sinf(joint_angles.hip_pitch) - L2 * cosf(joint_angles.hip_roll) * sinf(joint_angles.hip_pitch) * sinf(joint_angles.knee_pitch) + L1 * cosf(joint_angles.hip_yaw) * cosf(joint_angles.hip_pitch) * cosf(joint_angles.knee_pitch) * sinf(joint_angles.hip_roll) - L1 * cosf(joint_angles.hip_yaw) * sinf(joint_angles.hip_roll) * sinf(joint_angles.hip_pitch) * sinf(joint_angles.knee_pitch);
            break;
      case RIGHT_LEG_ID:
            center_pos.x = L1 - L2 * sinf(joint_angles.hip_roll) - L1 * cosf(joint_angles.hip_yaw) * cosf(joint_angles.hip_roll);
            center_pos.y = L4 * sinf(joint_angles.knee_pitch) - L1 * cosf(joint_angles.hip_pitch) * cosf(joint_angles.knee_pitch) * sinf(joint_angles.hip_yaw) + L2 * cosf(joint_angles.hip_roll) * cosf(joint_angles.hip_pitch) * sinf(joint_angles.knee_pitch) + L2 * cosf(joint_angles.hip_roll) * cosf(joint_angles.knee_pitch) * sinf(joint_angles.hip_pitch) + L1 * sinf(joint_angles.hip_yaw) * sinf(joint_angles.hip_pitch) * sinf(joint_angles.knee_pitch) - L1 * cosf(joint_angles.hip_yaw) * cosf(joint_angles.hip_pitch) * sinf(joint_angles.hip_roll) * sinf(joint_angles.knee_pitch) - L1 * cosf(joint_angles.hip_yaw) * cosf(joint_angles.knee_pitch) * sinf(joint_angles.hip_roll) * sinf(joint_angles.hip_pitch);
            center_pos.z = L4 * cosf(joint_angles.knee_pitch) - L4 - L2 + L2 * cosf(joint_angles.hip_roll) * cosf(joint_angles.hip_pitch) * cosf(joint_angles.knee_pitch) + L1 * cosf(joint_angles.hip_pitch) * sinf(joint_angles.hip_yaw) * sinf(joint_angles.knee_pitch) + L1 * cosf(joint_angles.knee_pitch) * sinf(joint_angles.hip_yaw) * sinf(joint_angles.hip_pitch) - L2 * cosf(joint_angles.hip_roll) * sinf(joint_angles.hip_pitch) * sinf(joint_angles.knee_pitch) - L1 * cosf(joint_angles.hip_yaw) * cosf(joint_angles.hip_pitch) * cosf(joint_angles.knee_pitch) * sinf(joint_angles.hip_roll) + L1 * cosf(joint_angles.hip_yaw) * sinf(joint_angles.hip_roll) * sinf(joint_angles.hip_pitch) * sinf(joint_angles.knee_pitch);
            break;
      }

      // Return the center positions
      return center_pos;
}

Joint_Angles Kinematics::computeCenterIK(const Direction_Vector &act_center_pos, const Direction_Vector &ref_center_pos, const Joint_Angles &act_joint_angles, const uint8_t &leg_id)
{
      // Initialize center positions
      Direction_Vector computed_center_pos;
      Eigen::Vector3f v_ref_center_pos(ref_center_pos.x, ref_center_pos.y, ref_center_pos.z);
      Eigen::Vector3f v_act_center_pos(act_center_pos.x, act_center_pos.y, act_center_pos.z);
      Eigen::Vector3f v_center_pos_error = v_ref_center_pos - v_act_center_pos;
      float IK_epsilon = v_center_pos_error.norm(); // Calculation will not be performed if the error is already less than the tolerance

      // Initialize the Jacobian matrix
      Eigen::Matrix3f m_Jacobian;

      // Initialize the joint angles
      Joint_Angles computed_joint_angles = act_joint_angles; // Initial condition for the joint angles is the current joint angles
      computed_joint_angles.hip_pitch = 0.3f;                // Initial condition for the hip pitch so that knee always bends in the forward direction
      Eigen::Vector3f v_delta_joint_angles;

      uint8_t IK_iteration_count = 0; // Reset the iteration count

      // Newton-Raphson method for inverse kinematics
      while (IK_epsilon > ERROR_TOLERANCE && IK_iteration_count < MAX_ITERATIONS)
      {
            // Initialize the Jacobian matrix based on the leg id
            switch (leg_id)
            {
            case LEFT_LEG_ID:
                  m_Jacobian(0, 0) = -L2 * cosf(computed_joint_angles.hip_roll) - L1 * cosf(computed_joint_angles.hip_yaw) * sinf(computed_joint_angles.hip_roll);
                  m_Jacobian(0, 1) = 0;
                  m_Jacobian(0, 2) = 0;
                  m_Jacobian(1, 0) = -sinf(computed_joint_angles.hip_pitch + computed_joint_angles.knee_pitch) * (L2 * sinf(computed_joint_angles.hip_roll) - L1 * cosf(computed_joint_angles.hip_yaw) * cosf(computed_joint_angles.hip_roll));
                  m_Jacobian(1, 1) = L2 * cosf(computed_joint_angles.hip_roll) * cosf(computed_joint_angles.hip_pitch) * cosf(computed_joint_angles.knee_pitch) - L1 * cosf(computed_joint_angles.hip_pitch) * sinf(computed_joint_angles.hip_yaw) * sinf(computed_joint_angles.knee_pitch) - L1 * cosf(computed_joint_angles.knee_pitch) * sinf(computed_joint_angles.hip_yaw) * sinf(computed_joint_angles.hip_pitch) - L2 * cosf(computed_joint_angles.hip_roll) * sinf(computed_joint_angles.hip_pitch) * sinf(computed_joint_angles.knee_pitch) + L1 * cosf(computed_joint_angles.hip_yaw) * cosf(computed_joint_angles.hip_pitch) * cosf(computed_joint_angles.knee_pitch) * sinf(computed_joint_angles.hip_roll) - L1 * cosf(computed_joint_angles.hip_yaw) * sinf(computed_joint_angles.hip_roll) * sinf(computed_joint_angles.hip_pitch) * sinf(computed_joint_angles.knee_pitch);
                  m_Jacobian(1, 2) = L4 * cosf(computed_joint_angles.knee_pitch) + L2 * cosf(computed_joint_angles.hip_roll) * cosf(computed_joint_angles.hip_pitch) * cosf(computed_joint_angles.knee_pitch) - L1 * cosf(computed_joint_angles.hip_pitch) * sinf(computed_joint_angles.hip_yaw) * sinf(computed_joint_angles.knee_pitch) - L1 * cosf(computed_joint_angles.knee_pitch) * sinf(computed_joint_angles.hip_yaw) * sinf(computed_joint_angles.hip_pitch) - L2 * cosf(computed_joint_angles.hip_roll) * sinf(computed_joint_angles.hip_pitch) * sinf(computed_joint_angles.knee_pitch) + L1 * cosf(computed_joint_angles.hip_yaw) * cosf(computed_joint_angles.hip_pitch) * cosf(computed_joint_angles.knee_pitch) * sinf(computed_joint_angles.hip_roll) - L1 * cosf(computed_joint_angles.hip_yaw) * sinf(computed_joint_angles.hip_roll) * sinf(computed_joint_angles.hip_pitch) * sinf(computed_joint_angles.knee_pitch);
                  m_Jacobian(2, 0) = -cosf(computed_joint_angles.hip_pitch + computed_joint_angles.knee_pitch) * (L2 * sinf(computed_joint_angles.hip_roll) - L1 * cosf(computed_joint_angles.hip_yaw) * cosf(computed_joint_angles.hip_roll));
                  m_Jacobian(2, 1) = L1 * sinf(computed_joint_angles.hip_yaw) * sinf(computed_joint_angles.hip_pitch) * sinf(computed_joint_angles.knee_pitch) - L2 * cosf(computed_joint_angles.hip_roll) * cosf(computed_joint_angles.hip_pitch) * sinf(computed_joint_angles.knee_pitch) - L2 * cosf(computed_joint_angles.hip_roll) * cosf(computed_joint_angles.knee_pitch) * sinf(computed_joint_angles.hip_pitch) - L1 * cosf(computed_joint_angles.hip_pitch) * cosf(computed_joint_angles.knee_pitch) * sinf(computed_joint_angles.hip_yaw) - L1 * cosf(computed_joint_angles.hip_yaw) * cosf(computed_joint_angles.hip_pitch) * sinf(computed_joint_angles.hip_roll) * sinf(computed_joint_angles.knee_pitch) - L1 * cosf(computed_joint_angles.hip_yaw) * cosf(computed_joint_angles.knee_pitch) * sinf(computed_joint_angles.hip_roll) * sinf(computed_joint_angles.hip_pitch);
                  m_Jacobian(2, 2) = L1 * sinf(computed_joint_angles.hip_yaw) * sinf(computed_joint_angles.hip_pitch) * sinf(computed_joint_angles.knee_pitch) - L1 * cosf(computed_joint_angles.hip_pitch) * cosf(computed_joint_angles.knee_pitch) * sinf(computed_joint_angles.hip_yaw) - L2 * cosf(computed_joint_angles.hip_roll) * cosf(computed_joint_angles.hip_pitch) * sinf(computed_joint_angles.knee_pitch) - L2 * cosf(computed_joint_angles.hip_roll) * cosf(computed_joint_angles.knee_pitch) * sinf(computed_joint_angles.hip_pitch) - L4 * sinf(computed_joint_angles.knee_pitch) - L1 * cosf(computed_joint_angles.hip_yaw) * cosf(computed_joint_angles.hip_pitch) * sinf(computed_joint_angles.hip_roll) * sinf(computed_joint_angles.knee_pitch) - L1 * cosf(computed_joint_angles.hip_yaw) * cosf(computed_joint_angles.knee_pitch) * sinf(computed_joint_angles.hip_roll) * sinf(computed_joint_angles.hip_pitch);
                  break;
            case RIGHT_LEG_ID:
                  m_Jacobian(0, 0) = L1 * cosf(computed_joint_angles.hip_yaw) * sinf(computed_joint_angles.hip_roll) - L2 * cosf(computed_joint_angles.hip_roll);
                  m_Jacobian(0, 1) = 0;
                  m_Jacobian(0, 2) = 0;
                  m_Jacobian(1, 0) = -sinf(computed_joint_angles.hip_pitch + computed_joint_angles.knee_pitch) * (L2 * sinf(computed_joint_angles.hip_roll) + L1 * cosf(computed_joint_angles.hip_yaw) * cosf(computed_joint_angles.hip_roll));
                  m_Jacobian(1, 1) = L2 * cosf(computed_joint_angles.hip_roll) * cosf(computed_joint_angles.hip_pitch) * cosf(computed_joint_angles.knee_pitch) + L1 * cosf(computed_joint_angles.hip_pitch) * sinf(computed_joint_angles.hip_yaw) * sinf(computed_joint_angles.knee_pitch) + L1 * cosf(computed_joint_angles.knee_pitch) * sinf(computed_joint_angles.hip_yaw) * sinf(computed_joint_angles.hip_pitch) - L2 * cosf(computed_joint_angles.hip_roll) * sinf(computed_joint_angles.hip_pitch) * sinf(computed_joint_angles.knee_pitch) - L1 * cosf(computed_joint_angles.hip_yaw) * cosf(computed_joint_angles.hip_pitch) * cosf(computed_joint_angles.knee_pitch) * sinf(computed_joint_angles.hip_roll) + L1 * cosf(computed_joint_angles.hip_yaw) * sinf(computed_joint_angles.hip_roll) * sinf(computed_joint_angles.hip_pitch) * sinf(computed_joint_angles.knee_pitch);
                  m_Jacobian(1, 2) = L4 * cosf(computed_joint_angles.knee_pitch) + L2 * cosf(computed_joint_angles.hip_roll) * cosf(computed_joint_angles.hip_pitch) * cosf(computed_joint_angles.knee_pitch) + L1 * cosf(computed_joint_angles.hip_pitch) * sinf(computed_joint_angles.hip_yaw) * sinf(computed_joint_angles.knee_pitch) + L1 * cosf(computed_joint_angles.knee_pitch) * sinf(computed_joint_angles.hip_yaw) * sinf(computed_joint_angles.hip_pitch) - L2 * cosf(computed_joint_angles.hip_roll) * sinf(computed_joint_angles.hip_pitch) * sinf(computed_joint_angles.knee_pitch) - L1 * cosf(computed_joint_angles.hip_yaw) * cosf(computed_joint_angles.hip_pitch) * cosf(computed_joint_angles.knee_pitch) * sinf(computed_joint_angles.hip_roll) + L1 * cosf(computed_joint_angles.hip_yaw) * sinf(computed_joint_angles.hip_roll) * sinf(computed_joint_angles.hip_pitch) * sinf(computed_joint_angles.knee_pitch);
                  m_Jacobian(2, 0) = -cosf(computed_joint_angles.hip_pitch + computed_joint_angles.knee_pitch) * (L2 * sinf(computed_joint_angles.hip_roll) + L1 * cosf(computed_joint_angles.hip_yaw) * cosf(computed_joint_angles.hip_roll));
                  m_Jacobian(2, 1) = L1 * cosf(computed_joint_angles.hip_pitch) * cosf(computed_joint_angles.knee_pitch) * sinf(computed_joint_angles.hip_yaw) - L2 * cosf(computed_joint_angles.hip_roll) * cosf(computed_joint_angles.hip_pitch) * sinf(computed_joint_angles.knee_pitch) - L2 * cosf(computed_joint_angles.hip_roll) * cosf(computed_joint_angles.knee_pitch) * sinf(computed_joint_angles.hip_pitch) - L1 * sinf(computed_joint_angles.hip_yaw) * sinf(computed_joint_angles.hip_pitch) * sinf(computed_joint_angles.knee_pitch) + L1 * cosf(computed_joint_angles.hip_yaw) * cosf(computed_joint_angles.hip_pitch) * sinf(computed_joint_angles.hip_roll) * sinf(computed_joint_angles.knee_pitch) + L1 * cosf(computed_joint_angles.hip_yaw) * cosf(computed_joint_angles.knee_pitch) * sinf(computed_joint_angles.hip_roll) * sinf(computed_joint_angles.hip_pitch);
                  m_Jacobian(2, 2) = L1 * cosf(computed_joint_angles.hip_pitch) * cosf(computed_joint_angles.knee_pitch) * sinf(computed_joint_angles.hip_yaw) - L4 * sinf(computed_joint_angles.knee_pitch) - L2 * cosf(computed_joint_angles.hip_roll) * cosf(computed_joint_angles.hip_pitch) * sinf(computed_joint_angles.knee_pitch) - L2 * cosf(computed_joint_angles.hip_roll) * cosf(computed_joint_angles.knee_pitch) * sinf(computed_joint_angles.hip_pitch) - L1 * sinf(computed_joint_angles.hip_yaw) * sinf(computed_joint_angles.hip_pitch) * sinf(computed_joint_angles.knee_pitch) + L1 * cosf(computed_joint_angles.hip_yaw) * cosf(computed_joint_angles.hip_pitch) * sinf(computed_joint_angles.hip_roll) * sinf(computed_joint_angles.knee_pitch) + L1 * cosf(computed_joint_angles.hip_yaw) * cosf(computed_joint_angles.knee_pitch) * sinf(computed_joint_angles.hip_roll) * sinf(computed_joint_angles.hip_pitch);
                  break;
            }
            // SR Inverse usinfg Levenberg-Marquardt method solved using ldlt decomposition as in this Ax = b, A is symmetric and positive definite
            v_delta_joint_angles = (m_Jacobian.transpose() * m_Jacobian + IK_LAMBDA * Eigen::Matrix3f::Identity()).ldlt().solve(m_Jacobian.transpose() * v_center_pos_error);

            // Update the joint angles
            computed_joint_angles.hip_roll += v_delta_joint_angles(0);
            computed_joint_angles.hip_pitch += v_delta_joint_angles(1);
            computed_joint_angles.knee_pitch += v_delta_joint_angles(2);

            // Recompute the center position
            computed_center_pos = computeCenterFK(computed_joint_angles, leg_id);
            v_act_center_pos = {computed_center_pos.x, computed_center_pos.y, computed_center_pos.z};
            v_center_pos_error = v_ref_center_pos - v_act_center_pos;

            IK_epsilon = v_center_pos_error.norm(); // Compute the error norm to check for convergence

            IK_iteration_count++; // Increment the iteration count
      }

      computed_joint_angles.hip_yaw = 0; // Hip yaw is always zero

      return computed_joint_angles;
}
