#include "kinematics.h"
#include "Eigen/Dense"
#include <cmath>

Direction_Vector Kinematics::computeCenterOfMass(const Joint_Angles &joint_angles_left, const Joint_Angles &joint_angles_right, const FusionMatrix &rotation_matrix)
{
      // Compute the CoM of center part
      CoM_link1.x = C1.x;
      CoM_link1.y = C1.y;
      CoM_link1.z = C1.z;

      // Compute the center of mass of the left leg parts
      CoM_link2.x = C2.x * cosf(joint_angles_left.hip_yaw) - L1 + C2.y * sinf(joint_angles_left.hip_yaw);
      CoM_link2.y = C2.y * cosf(joint_angles_left.hip_yaw) - C2.x * sinf(joint_angles_left.hip_yaw);
      CoM_link2.z = C2.z;

      CoM_link3.x = C3.y * sinf(joint_angles_left.hip_yaw) - L1 - L3 * sinf(joint_angles_left.hip_yaw) + C3.x * cosf(joint_angles_left.hip_yaw) * cosf(joint_angles_left.hip_roll) + C3.z * cosf(joint_angles_left.hip_yaw) * sinf(joint_angles_left.hip_roll);
      CoM_link3.y = C3.y * cosf(joint_angles_left.hip_yaw) - L3 * cosf(joint_angles_left.hip_yaw) - C3.x * cosf(joint_angles_left.hip_roll) * sinf(joint_angles_left.hip_yaw) - C3.z * sinf(joint_angles_left.hip_yaw) * sinf(joint_angles_left.hip_roll);
      CoM_link3.z = C3.z * cosf(joint_angles_left.hip_roll) - L2 - C3.x * sinf(joint_angles_left.hip_roll);

      CoM_link4.x = C4.x * cosf(joint_angles_left.hip_yaw) * cosf(joint_angles_left.hip_roll) - L1 + C4.y * cosf(joint_angles_left.hip_pitch) * sinf(joint_angles_left.hip_yaw) - C4.z * sinf(joint_angles_left.hip_yaw) * sinf(joint_angles_left.hip_pitch) + C4.z * cosf(joint_angles_left.hip_yaw) * cosf(joint_angles_left.hip_pitch) * sinf(joint_angles_left.hip_roll) + C4.y * cosf(joint_angles_left.hip_yaw) * sinf(joint_angles_left.hip_roll) * sinf(joint_angles_left.hip_pitch);
      CoM_link4.y = C4.y * cosf(joint_angles_left.hip_yaw) * cosf(joint_angles_left.hip_pitch) - C4.x * cosf(joint_angles_left.hip_roll) * sinf(joint_angles_left.hip_yaw) - C4.z * cosf(joint_angles_left.hip_yaw) * sinf(joint_angles_left.hip_pitch) - C4.z * cosf(joint_angles_left.hip_pitch) * sinf(joint_angles_left.hip_yaw) * sinf(joint_angles_left.hip_roll) - C4.y * sinf(joint_angles_left.hip_yaw) * sinf(joint_angles_left.hip_roll) * sinf(joint_angles_left.hip_pitch);
      CoM_link4.z = C4.z * cosf(joint_angles_left.hip_roll) * cosf(joint_angles_left.hip_pitch) - C4.x * sinf(joint_angles_left.hip_roll) - L2 + C4.y * cosf(joint_angles_left.hip_roll) * sinf(joint_angles_left.hip_pitch);

      CoM_link5.x = C5.x * cosf(joint_angles_left.hip_yaw) * cosf(joint_angles_left.hip_roll) - L1 + L4 * sinf(joint_angles_left.hip_yaw) * sinf(joint_angles_left.hip_pitch) + C5.y * cosf(joint_angles_left.hip_pitch) * cosf(joint_angles_left.knee_pitch) * sinf(joint_angles_left.hip_yaw) - L4 * cosf(joint_angles_left.hip_yaw) * cosf(joint_angles_left.hip_pitch) * sinf(joint_angles_left.hip_roll) - C5.z * cosf(joint_angles_left.hip_pitch) * sinf(joint_angles_left.hip_yaw) * sinf(joint_angles_left.knee_pitch) - C5.z * cosf(joint_angles_left.knee_pitch) * sinf(joint_angles_left.hip_yaw) * sinf(joint_angles_left.hip_pitch) - C5.y * sinf(joint_angles_left.hip_yaw) * sinf(joint_angles_left.hip_pitch) * sinf(joint_angles_left.knee_pitch) + C5.z * cosf(joint_angles_left.hip_yaw) * cosf(joint_angles_left.hip_pitch) * cosf(joint_angles_left.knee_pitch) * sinf(joint_angles_left.hip_roll) + C5.y * cosf(joint_angles_left.hip_yaw) * cosf(joint_angles_left.hip_pitch) * sinf(joint_angles_left.hip_roll) * sinf(joint_angles_left.knee_pitch) + C5.y * cosf(joint_angles_left.hip_yaw) * cosf(joint_angles_left.knee_pitch) * sinf(joint_angles_left.hip_roll) * sinf(joint_angles_left.hip_pitch) - C5.z * cosf(joint_angles_left.hip_yaw) * sinf(joint_angles_left.hip_roll) * sinf(joint_angles_left.hip_pitch) * sinf(joint_angles_left.knee_pitch);
      CoM_link5.y = L4 * cosf(joint_angles_left.hip_yaw) * sinf(joint_angles_left.hip_pitch) - C5.x * cosf(joint_angles_left.hip_roll) * sinf(joint_angles_left.hip_yaw) + C5.y * cosf(joint_angles_left.hip_yaw) * cosf(joint_angles_left.hip_pitch) * cosf(joint_angles_left.knee_pitch) - C5.z * cosf(joint_angles_left.hip_yaw) * cosf(joint_angles_left.hip_pitch) * sinf(joint_angles_left.knee_pitch) - C5.z * cosf(joint_angles_left.hip_yaw) * cosf(joint_angles_left.knee_pitch) * sinf(joint_angles_left.hip_pitch) - C5.y * cosf(joint_angles_left.hip_yaw) * sinf(joint_angles_left.hip_pitch) * sinf(joint_angles_left.knee_pitch) + L4 * cosf(joint_angles_left.hip_pitch) * sinf(joint_angles_left.hip_yaw) * sinf(joint_angles_left.hip_roll) - C5.z * cosf(joint_angles_left.hip_pitch) * cosf(joint_angles_left.knee_pitch) * sinf(joint_angles_left.hip_yaw) * sinf(joint_angles_left.hip_roll) - C5.y * cosf(joint_angles_left.hip_pitch) * sinf(joint_angles_left.hip_yaw) * sinf(joint_angles_left.hip_roll) * sinf(joint_angles_left.knee_pitch) - C5.y * cosf(joint_angles_left.knee_pitch) * sinf(joint_angles_left.hip_yaw) * sinf(joint_angles_left.hip_roll) * sinf(joint_angles_left.hip_pitch) + C5.z * sinf(joint_angles_left.hip_yaw) * sinf(joint_angles_left.hip_roll) * sinf(joint_angles_left.hip_pitch) * sinf(joint_angles_left.knee_pitch);
      CoM_link5.z = C5.z * cosf(joint_angles_left.hip_roll) * cosf(joint_angles_left.hip_pitch) * cosf(joint_angles_left.knee_pitch) - C5.x * sinf(joint_angles_left.hip_roll) - L4 * cosf(joint_angles_left.hip_roll) * cosf(joint_angles_left.hip_pitch) - L2 + C5.y * cosf(joint_angles_left.hip_roll) * cosf(joint_angles_left.hip_pitch) * sinf(joint_angles_left.knee_pitch) + C5.y * cosf(joint_angles_left.hip_roll) * cosf(joint_angles_left.knee_pitch) * sinf(joint_angles_left.hip_pitch) - C5.z * cosf(joint_angles_left.hip_roll) * sinf(joint_angles_left.hip_pitch) * sinf(joint_angles_left.knee_pitch);

      // Compute the center of mass of the right leg parts
      CoM_link6.x = L1 - C2.x * cosf(joint_angles_right.hip_yaw) + C2.y * sinf(joint_angles_right.hip_yaw);
      CoM_link6.y = C2.y * cosf(joint_angles_right.hip_yaw) + C2.x * sinf(joint_angles_right.hip_yaw);
      CoM_link6.z = C2.z;

      CoM_link7.x = L1 + C3.y * sinf(joint_angles_right.hip_yaw) - L3 * sinf(joint_angles_right.hip_yaw) + C3.x * cosf(joint_angles_right.hip_yaw) * cosf(joint_angles_right.hip_roll) + C3.z * cosf(joint_angles_right.hip_yaw) * sinf(joint_angles_right.hip_roll);
      CoM_link7.y = C3.y * cosf(joint_angles_right.hip_yaw) - L3 * cosf(joint_angles_right.hip_yaw) - C3.x * cosf(joint_angles_right.hip_roll) * sinf(joint_angles_right.hip_yaw) - C3.z * sinf(joint_angles_right.hip_yaw) * sinf(joint_angles_right.hip_roll);
      CoM_link7.z = C3.z * cosf(joint_angles_right.hip_roll) - L2 - C3.x * sinf(joint_angles_right.hip_roll);

      CoM_link8.x = L1 + C4.x * cosf(joint_angles_right.hip_yaw) * cosf(joint_angles_right.hip_roll) + C4.y * cosf(joint_angles_right.hip_pitch) * sinf(joint_angles_right.hip_yaw) - C4.z * sinf(joint_angles_right.hip_yaw) * sinf(joint_angles_right.hip_pitch) + C4.z * cosf(joint_angles_right.hip_yaw) * cosf(joint_angles_right.hip_pitch) * sinf(joint_angles_right.hip_roll) + C4.y * cosf(joint_angles_right.hip_yaw) * sinf(joint_angles_right.hip_roll) * sinf(joint_angles_right.hip_pitch);
      CoM_link8.y = C4.y * cosf(joint_angles_right.hip_yaw) * cosf(joint_angles_right.hip_pitch) - C4.x * cosf(joint_angles_right.hip_roll) * sinf(joint_angles_right.hip_yaw) - C4.z * cosf(joint_angles_right.hip_yaw) * sinf(joint_angles_right.hip_pitch) - C4.z * cosf(joint_angles_right.hip_pitch) * sinf(joint_angles_right.hip_yaw) * sinf(joint_angles_right.hip_roll) - C4.y * sinf(joint_angles_right.hip_yaw) * sinf(joint_angles_right.hip_roll) * sinf(joint_angles_right.hip_pitch);
      CoM_link8.z = C4.z * cosf(joint_angles_right.hip_roll) * cosf(joint_angles_right.hip_pitch) - C4.x * sinf(joint_angles_right.hip_roll) - L2 + C4.y * cosf(joint_angles_right.hip_roll) * sinf(joint_angles_right.hip_pitch);

      CoM_link9.x = L1 + C5.x * cosf(joint_angles_right.hip_yaw) * cosf(joint_angles_right.hip_roll) + L4 * sinf(joint_angles_right.hip_yaw) * sinf(joint_angles_right.hip_pitch) + C5.y * cosf(joint_angles_right.hip_pitch) * cosf(joint_angles_right.knee_pitch) * sinf(joint_angles_right.hip_yaw) - L4 * cosf(joint_angles_right.hip_yaw) * cosf(joint_angles_right.hip_pitch) * sinf(joint_angles_right.hip_roll) - C5.z * cosf(joint_angles_right.hip_pitch) * sinf(joint_angles_right.hip_yaw) * sinf(joint_angles_right.knee_pitch) - C5.z * cosf(joint_angles_right.knee_pitch) * sinf(joint_angles_right.hip_yaw) * sinf(joint_angles_right.hip_pitch) - C5.y * sinf(joint_angles_right.hip_yaw) * sinf(joint_angles_right.hip_pitch) * sinf(joint_angles_right.knee_pitch) + C5.z * cosf(joint_angles_right.hip_yaw) * cosf(joint_angles_right.hip_pitch) * cosf(joint_angles_right.knee_pitch) * sinf(joint_angles_right.hip_roll) + C5.y * cosf(joint_angles_right.hip_yaw) * cosf(joint_angles_right.hip_pitch) * sinf(joint_angles_right.hip_roll) * sinf(joint_angles_right.knee_pitch) + C5.y * cosf(joint_angles_right.hip_yaw) * cosf(joint_angles_right.knee_pitch) * sinf(joint_angles_right.hip_roll) * sinf(joint_angles_right.hip_pitch) - C5.z * cosf(joint_angles_right.hip_yaw) * sinf(joint_angles_right.hip_roll) * sinf(joint_angles_right.hip_pitch) * sinf(joint_angles_right.knee_pitch);
      CoM_link9.y = L4 * cosf(joint_angles_right.hip_yaw) * sinf(joint_angles_right.hip_pitch) - C5.x * cosf(joint_angles_right.hip_roll) * sinf(joint_angles_right.hip_yaw) + C5.y * cosf(joint_angles_right.hip_yaw) * cosf(joint_angles_right.hip_pitch) * cosf(joint_angles_right.knee_pitch) - C5.z * cosf(joint_angles_right.hip_yaw) * cosf(joint_angles_right.hip_pitch) * sinf(joint_angles_right.knee_pitch) - C5.z * cosf(joint_angles_right.hip_yaw) * cosf(joint_angles_right.knee_pitch) * sinf(joint_angles_right.hip_pitch) - C5.y * cosf(joint_angles_right.hip_yaw) * sinf(joint_angles_right.hip_pitch) * sinf(joint_angles_right.knee_pitch) + L4 * cosf(joint_angles_right.hip_pitch) * sinf(joint_angles_right.hip_yaw) * sinf(joint_angles_right.hip_roll) - C5.z * cosf(joint_angles_right.hip_pitch) * cosf(joint_angles_right.knee_pitch) * sinf(joint_angles_right.hip_yaw) * sinf(joint_angles_right.hip_roll) - C5.y * cosf(joint_angles_right.hip_pitch) * sinf(joint_angles_right.hip_yaw) * sinf(joint_angles_right.hip_roll) * sinf(joint_angles_right.knee_pitch) - C5.y * cosf(joint_angles_right.knee_pitch) * sinf(joint_angles_right.hip_yaw) * sinf(joint_angles_right.hip_roll) * sinf(joint_angles_right.hip_pitch) + C5.z * sinf(joint_angles_right.hip_yaw) * sinf(joint_angles_right.hip_roll) * sinf(joint_angles_right.hip_pitch) * sinf(joint_angles_right.knee_pitch);
      CoM_link9.z = C5.z * cosf(joint_angles_right.hip_roll) * cosf(joint_angles_right.hip_pitch) * cosf(joint_angles_right.knee_pitch) - C5.x * sinf(joint_angles_right.hip_roll) - L4 * cosf(joint_angles_right.hip_roll) * cosf(joint_angles_right.hip_pitch) - L2 + C5.y * cosf(joint_angles_right.hip_roll) * cosf(joint_angles_right.hip_pitch) * sinf(joint_angles_right.knee_pitch) + C5.y * cosf(joint_angles_right.hip_roll) * cosf(joint_angles_right.knee_pitch) * sinf(joint_angles_right.hip_pitch) - C5.z * cosf(joint_angles_right.hip_roll) * sinf(joint_angles_right.hip_pitch) * sinf(joint_angles_right.knee_pitch);

      // Compute the center of mass of the robot
      CoM_body_frame.x = (CoM_link1.x * M1 + CoM_link2.x * M2 + CoM_link3.x * M3 + CoM_link4.x * M4 + CoM_link5.x * M5 + CoM_link6.x * M2 + CoM_link7.x * M3 + CoM_link8.x * M4 + CoM_link9.x * M5) / MT;
      CoM_body_frame.y = (CoM_link1.y * M1 + CoM_link2.y * M2 + CoM_link3.y * M3 + CoM_link4.y * M4 + CoM_link5.y * M5 + CoM_link6.y * M2 + CoM_link7.y * M3 + CoM_link8.y * M4 + CoM_link9.y * M5) / MT;
      CoM_body_frame.z = (CoM_link1.z * M1 + CoM_link2.z * M2 + CoM_link3.z * M3 + CoM_link4.z * M4 + CoM_link5.z * M5 + CoM_link6.z * M2 + CoM_link7.z * M3 + CoM_link8.z * M4 + CoM_link9.z * M5) / MT;

      // Rotate the center of mass of the robot from body frame to world frame
      Direction_Vector CoM_world_frame;

      CoM_world_frame.x = (CoM_body_frame.x * rotation_matrix.array[0][0] + CoM_body_frame.y * rotation_matrix.array[0][1] + CoM_body_frame.z * rotation_matrix.array[0][2]);
      CoM_world_frame.y = (CoM_body_frame.x * rotation_matrix.array[1][0] + CoM_body_frame.y * rotation_matrix.array[1][1] + CoM_body_frame.z * rotation_matrix.array[1][2]);
      CoM_world_frame.z = CoM_body_frame.x * rotation_matrix.array[2][0] + CoM_body_frame.y * rotation_matrix.array[2][1] + CoM_body_frame.z * rotation_matrix.array[2][2];

      return CoM_world_frame;
}

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
