#include "Kinematics.h"
#include <Eigen/Dense>
#include <cmath>
#include "User_Math.h"
#include "main.h"

Kinematics::Kinematics()
{
      // Initialize inverse kinematics joint angles
      IK_left_leg_angles = {0.0f, 0.0f, 0.0f, 0.0f};
      IK_right_leg_angles = {0.0f, 0.0f, 0.0f, 0.0f};
      // Initialize forward kinematics foot positions
      FK_left_foot_pos = {0.0f, 0.0f, 0.0f};
      FK_right_foot_pos = {0.0f, 0.0f, 0.0f};
}

Direction_Vector Kinematics::computeForwardKinematics(Joint_Angle joint_angles, const int leg)
{
      Direction_Vector foot_pos;
      const DH_Parameter *dh_param;
      if (leg == LEFT_LEG)
      {
            dh_param = DH_Left_Leg;
      }
      else if (leg == RIGHT_LEG)
      {
            dh_param = DH_Right_Leg;
      }
      else
      {
            // Invalid leg identifier
            return foot_pos = {0.0f, 0.0f, 0.0f};
      }

      foot_pos.x = dh_param[0].a + dh_param[3].a * (cosf(joint_angles.hip_pitch) * (cosf(joint_angles.hip_yaw) * cosf(joint_angles.hip_roll) - cosf(dh_param[1].alpha) * sinf(joint_angles.hip_yaw) * sinf(joint_angles.hip_roll)) - cosf(dh_param[2].alpha) * sinf(joint_angles.hip_pitch) * (cosf(joint_angles.hip_yaw) * sinf(joint_angles.hip_roll) + cosf(dh_param[1].alpha) * cosf(joint_angles.hip_roll) * sinf(joint_angles.hip_yaw)) + sinf(dh_param[1].alpha) * sinf(dh_param[2].alpha) * sinf(joint_angles.hip_yaw) * sinf(joint_angles.hip_pitch)) + dh_param[4].a * (cosf(joint_angles.knee_pitch) * (cosf(joint_angles.hip_pitch) * (cosf(joint_angles.hip_yaw) * cosf(joint_angles.hip_roll) - cosf(dh_param[1].alpha) * sinf(joint_angles.hip_yaw) * sinf(joint_angles.hip_roll)) - cosf(dh_param[2].alpha) * sinf(joint_angles.hip_pitch) * (cosf(joint_angles.hip_yaw) * sinf(joint_angles.hip_roll) + cosf(dh_param[1].alpha) * cosf(joint_angles.hip_roll) * sinf(joint_angles.hip_yaw)) + sinf(dh_param[1].alpha) * sinf(dh_param[2].alpha) * sinf(joint_angles.hip_yaw) * sinf(joint_angles.hip_pitch)) - sinf(joint_angles.knee_pitch) * (sinf(joint_angles.hip_pitch) * (cosf(joint_angles.hip_yaw) * cosf(joint_angles.hip_roll) - cosf(dh_param[1].alpha) * sinf(joint_angles.hip_yaw) * sinf(joint_angles.hip_roll)) + cosf(dh_param[2].alpha) * cosf(joint_angles.hip_pitch) * (cosf(joint_angles.hip_yaw) * sinf(joint_angles.hip_roll) + cosf(dh_param[1].alpha) * cosf(joint_angles.hip_roll) * sinf(joint_angles.hip_yaw)) - sinf(dh_param[1].alpha) * sinf(dh_param[2].alpha) * cosf(joint_angles.hip_pitch) * sinf(joint_angles.hip_yaw))) + dh_param[2].a * (cosf(joint_angles.hip_yaw) * cosf(joint_angles.hip_roll) - cosf(dh_param[1].alpha) * sinf(joint_angles.hip_yaw) * sinf(joint_angles.hip_roll)) + dh_param[1].a * cosf(joint_angles.hip_yaw);
      foot_pos.y = dh_param[2].a * (cosf(joint_angles.hip_roll) * sinf(joint_angles.hip_yaw) + cosf(dh_param[1].alpha) * cosf(joint_angles.hip_yaw) * sinf(joint_angles.hip_roll)) + dh_param[1].a * sinf(joint_angles.hip_yaw) - dh_param[4].a * (sinf(joint_angles.knee_pitch) * (sinf(joint_angles.hip_pitch) * (cosf(joint_angles.hip_roll) * sinf(joint_angles.hip_yaw) + cosf(dh_param[1].alpha) * cosf(joint_angles.hip_yaw) * sinf(joint_angles.hip_roll)) + cosf(dh_param[2].alpha) * cosf(joint_angles.hip_pitch) * (sinf(joint_angles.hip_yaw) * sinf(joint_angles.hip_roll) - cosf(dh_param[1].alpha) * cosf(joint_angles.hip_yaw) * cosf(joint_angles.hip_roll)) + sinf(dh_param[1].alpha) * sinf(dh_param[2].alpha) * cosf(joint_angles.hip_yaw) * cosf(joint_angles.hip_pitch)) + cosf(joint_angles.knee_pitch) * (cosf(dh_param[2].alpha) * sinf(joint_angles.hip_pitch) * (sinf(joint_angles.hip_yaw) * sinf(joint_angles.hip_roll) - cosf(dh_param[1].alpha) * cosf(joint_angles.hip_yaw) * cosf(joint_angles.hip_roll)) - cosf(joint_angles.hip_pitch) * (cosf(joint_angles.hip_roll) * sinf(joint_angles.hip_yaw) + cosf(dh_param[1].alpha) * cosf(joint_angles.hip_yaw) * sinf(joint_angles.hip_roll)) + sinf(dh_param[1].alpha) * sinf(dh_param[2].alpha) * cosf(joint_angles.hip_yaw) * sinf(joint_angles.hip_pitch))) - dh_param[3].a * (cosf(dh_param[2].alpha) * sinf(joint_angles.hip_pitch) * (sinf(joint_angles.hip_yaw) * sinf(joint_angles.hip_roll) - cosf(dh_param[1].alpha) * cosf(joint_angles.hip_yaw) * cosf(joint_angles.hip_roll)) - cosf(joint_angles.hip_pitch) * (cosf(joint_angles.hip_roll) * sinf(joint_angles.hip_yaw) + cosf(dh_param[1].alpha) * cosf(joint_angles.hip_yaw) * sinf(joint_angles.hip_roll)) + sinf(dh_param[1].alpha) * sinf(dh_param[2].alpha) * cosf(joint_angles.hip_yaw) * sinf(joint_angles.hip_pitch));
      foot_pos.z = dh_param[4].a * (cosf(joint_angles.knee_pitch) * (cosf(dh_param[1].alpha) * sinf(dh_param[2].alpha) * sinf(joint_angles.hip_pitch) + sinf(dh_param[1].alpha) * cosf(joint_angles.hip_pitch) * sinf(joint_angles.hip_roll) + cosf(dh_param[2].alpha) * sinf(dh_param[1].alpha) * cosf(joint_angles.hip_roll) * sinf(joint_angles.hip_pitch)) + sinf(joint_angles.knee_pitch) * (cosf(dh_param[1].alpha) * sinf(dh_param[2].alpha) * cosf(joint_angles.hip_pitch) - sinf(dh_param[1].alpha) * sinf(joint_angles.hip_roll) * sinf(joint_angles.hip_pitch) + cosf(dh_param[2].alpha) * sinf(dh_param[1].alpha) * cosf(joint_angles.hip_roll) * cosf(joint_angles.hip_pitch))) + dh_param[3].a * (cosf(dh_param[1].alpha) * sinf(dh_param[2].alpha) * sinf(joint_angles.hip_pitch) + sinf(dh_param[1].alpha) * cosf(joint_angles.hip_pitch) * sinf(joint_angles.hip_roll) + cosf(dh_param[2].alpha) * sinf(dh_param[1].alpha) * cosf(joint_angles.hip_roll) * sinf(joint_angles.hip_pitch)) + dh_param[2].a * sinf(dh_param[1].alpha) * sinf(joint_angles.hip_roll);

      // Return the foot positions
      return foot_pos;
}

Joint_Angle Kinematics::computeInverseKinematics(const Direction_Vector &ref_foot_pos, const Direction_Vector &act_foot_pos, const Joint_Angle &joint_angles, const int leg)
{
      // Initialize foot positions
      Eigen::Matrix<float, 3, 1> v_ref_foot_pos(ref_foot_pos.x, ref_foot_pos.y, ref_foot_pos.z);
      Eigen::Matrix<float, 3, 1> v_act_foot_pos(act_foot_pos.x, act_foot_pos.y, act_foot_pos.z);

      // Initialize the joint angle guess to be the current joint angles
      Eigen::Matrix<float, 4, 1> v_ref_joint_angles(joint_angles.hip_yaw, joint_angles.hip_roll + LEFT_LEG_HIP_ROLL_OFFSET, joint_angles.hip_pitch, joint_angles.knee_pitch);

      // Select the DH parameters based on the leg
      const DH_Parameter *dh_param;
      if (leg == LEFT_LEG)
      {
            dh_param = DH_Left_Leg;
      }
      else if (leg == RIGHT_LEG)
      {
            dh_param = DH_Right_Leg;
      }
      else
      {
            // Invalid leg identifier
            return IK_left_leg_angles = {0.0f, 0.0f, 0.0f, 0.0f};
      }

      // Initialize the Jacobian and Inverse Jacobian matrix
      Eigen::Matrix<float, 3, 3> m_jacobian;
      Eigen::Matrix<float, 3, 3> m_inv_jacobian;

      Eigen::Matrix<float, 3, 1> v_foot_pos_error = v_ref_foot_pos - v_act_foot_pos;
      Eigen::Matrix<float, 3, 1> v_delta_joint_angles;

      IK_iteration_count = 0;
      IK_epsilon = 1.0f;

      // Newton-Raphson method for inverse kinematics
      while (IK_epsilon > UPDATE_TOLERANCE && v_foot_pos_error.norm() > ERROR_TOLERANCE && IK_iteration_count < MAX_ITERATIONS)
      {
            m_jacobian(0, 0) = -dh_param[2].a * (cos(v_ref_joint_angles(0)) * sin(v_ref_joint_angles(1)) + cos(dh_param[1].alpha) * cos(v_ref_joint_angles(1)) * sin(v_ref_joint_angles(0))) - dh_param[3].a * (cos(v_ref_joint_angles(2)) * (cos(v_ref_joint_angles(0)) * sin(v_ref_joint_angles(1)) + cos(dh_param[1].alpha) * cos(v_ref_joint_angles(1)) * sin(v_ref_joint_angles(0))) + cos(dh_param[2].alpha) * sin(v_ref_joint_angles(2)) * (cos(v_ref_joint_angles(0)) * cos(v_ref_joint_angles(1)) - cos(dh_param[1].alpha) * sin(v_ref_joint_angles(0)) * sin(v_ref_joint_angles(1)))) - dh_param[4].a * (cos(v_ref_joint_angles(3)) * (cos(v_ref_joint_angles(2)) * (cos(v_ref_joint_angles(0)) * sin(v_ref_joint_angles(1)) + cos(dh_param[1].alpha) * cos(v_ref_joint_angles(1)) * sin(v_ref_joint_angles(0))) + cos(dh_param[2].alpha) * sin(v_ref_joint_angles(2)) * (cos(v_ref_joint_angles(0)) * cos(v_ref_joint_angles(1)) - cos(dh_param[1].alpha) * sin(v_ref_joint_angles(0)) * sin(v_ref_joint_angles(1)))) - sin(v_ref_joint_angles(3)) * (sin(v_ref_joint_angles(2)) * (cos(v_ref_joint_angles(0)) * sin(v_ref_joint_angles(1)) + cos(dh_param[1].alpha) * cos(v_ref_joint_angles(1)) * sin(v_ref_joint_angles(0))) - cos(dh_param[2].alpha) * cos(v_ref_joint_angles(2)) * (cos(v_ref_joint_angles(0)) * cos(v_ref_joint_angles(1)) - cos(dh_param[1].alpha) * sin(v_ref_joint_angles(0)) * sin(v_ref_joint_angles(1)))));
            m_jacobian(0, 1) = -dh_param[4].a * (cos(v_ref_joint_angles(3)) * (sin(v_ref_joint_angles(2)) * (cos(v_ref_joint_angles(0)) * cos(v_ref_joint_angles(1)) - cos(dh_param[1].alpha) * sin(v_ref_joint_angles(0)) * sin(v_ref_joint_angles(1))) + cos(dh_param[2].alpha) * cos(v_ref_joint_angles(2)) * (cos(v_ref_joint_angles(0)) * sin(v_ref_joint_angles(1)) + cos(dh_param[1].alpha) * cos(v_ref_joint_angles(1)) * sin(v_ref_joint_angles(0))) - sin(dh_param[1].alpha) * sin(dh_param[2].alpha) * cos(v_ref_joint_angles(2)) * sin(v_ref_joint_angles(0))) + sin(v_ref_joint_angles(3)) * (cos(v_ref_joint_angles(2)) * (cos(v_ref_joint_angles(0)) * cos(v_ref_joint_angles(1)) - cos(dh_param[1].alpha) * sin(v_ref_joint_angles(0)) * sin(v_ref_joint_angles(1))) - cos(dh_param[2].alpha) * sin(v_ref_joint_angles(2)) * (cos(v_ref_joint_angles(0)) * sin(v_ref_joint_angles(1)) + cos(dh_param[1].alpha) * cos(v_ref_joint_angles(1)) * sin(v_ref_joint_angles(0))) + sin(dh_param[1].alpha) * sin(dh_param[2].alpha) * sin(v_ref_joint_angles(0)) * sin(v_ref_joint_angles(2)))) - dh_param[3].a * (sin(v_ref_joint_angles(2)) * (cos(v_ref_joint_angles(0)) * cos(v_ref_joint_angles(1)) - cos(dh_param[1].alpha) * sin(v_ref_joint_angles(0)) * sin(v_ref_joint_angles(1))) + cos(dh_param[2].alpha) * cos(v_ref_joint_angles(2)) * (cos(v_ref_joint_angles(0)) * sin(v_ref_joint_angles(1)) + cos(dh_param[1].alpha) * cos(v_ref_joint_angles(1)) * sin(v_ref_joint_angles(0))) - sin(dh_param[1].alpha) * sin(dh_param[2].alpha) * cos(v_ref_joint_angles(2)) * sin(v_ref_joint_angles(0)));
            m_jacobian(0, 2) = -dh_param[4].a * (cos(v_ref_joint_angles(3)) * (sin(v_ref_joint_angles(2)) * (cos(v_ref_joint_angles(0)) * cos(v_ref_joint_angles(1)) - cos(dh_param[1].alpha) * sin(v_ref_joint_angles(0)) * sin(v_ref_joint_angles(1))) + cos(dh_param[2].alpha) * cos(v_ref_joint_angles(2)) * (cos(v_ref_joint_angles(0)) * sin(v_ref_joint_angles(1)) + cos(dh_param[1].alpha) * cos(v_ref_joint_angles(1)) * sin(v_ref_joint_angles(0))) - sin(dh_param[1].alpha) * sin(dh_param[2].alpha) * cos(v_ref_joint_angles(2)) * sin(v_ref_joint_angles(0))) + sin(v_ref_joint_angles(3)) * (cos(v_ref_joint_angles(2)) * (cos(v_ref_joint_angles(0)) * cos(v_ref_joint_angles(1)) - cos(dh_param[1].alpha) * sin(v_ref_joint_angles(0)) * sin(v_ref_joint_angles(1))) - cos(dh_param[2].alpha) * sin(v_ref_joint_angles(2)) * (cos(v_ref_joint_angles(0)) * sin(v_ref_joint_angles(1)) + cos(dh_param[1].alpha) * cos(v_ref_joint_angles(1)) * sin(v_ref_joint_angles(0))) + sin(dh_param[1].alpha) * sin(dh_param[2].alpha) * sin(v_ref_joint_angles(0)) * sin(v_ref_joint_angles(2))));
            m_jacobian(1, 0) = -dh_param[2].a * (sin(v_ref_joint_angles(0)) * sin(v_ref_joint_angles(1)) - cos(dh_param[1].alpha) * cos(v_ref_joint_angles(0)) * cos(v_ref_joint_angles(1))) - dh_param[3].a * (cos(v_ref_joint_angles(2)) * (sin(v_ref_joint_angles(0)) * sin(v_ref_joint_angles(1)) - cos(dh_param[1].alpha) * cos(v_ref_joint_angles(0)) * cos(v_ref_joint_angles(1))) + cos(dh_param[2].alpha) * sin(v_ref_joint_angles(2)) * (cos(v_ref_joint_angles(1)) * sin(v_ref_joint_angles(0)) + cos(dh_param[1].alpha) * cos(v_ref_joint_angles(0)) * sin(v_ref_joint_angles(1)))) - dh_param[4].a * (cos(v_ref_joint_angles(3)) * (cos(v_ref_joint_angles(2)) * (sin(v_ref_joint_angles(0)) * sin(v_ref_joint_angles(1)) - cos(dh_param[1].alpha) * cos(v_ref_joint_angles(0)) * cos(v_ref_joint_angles(1))) + cos(dh_param[2].alpha) * sin(v_ref_joint_angles(2)) * (cos(v_ref_joint_angles(1)) * sin(v_ref_joint_angles(0)) + cos(dh_param[1].alpha) * cos(v_ref_joint_angles(0)) * sin(v_ref_joint_angles(1)))) - sin(v_ref_joint_angles(3)) * (sin(v_ref_joint_angles(2)) * (sin(v_ref_joint_angles(0)) * sin(v_ref_joint_angles(1)) - cos(dh_param[1].alpha) * cos(v_ref_joint_angles(0)) * cos(v_ref_joint_angles(1))) - cos(dh_param[2].alpha) * cos(v_ref_joint_angles(2)) * (cos(v_ref_joint_angles(1)) * sin(v_ref_joint_angles(0)) + cos(dh_param[1].alpha) * cos(v_ref_joint_angles(0)) * sin(v_ref_joint_angles(1)))));
            m_jacobian(1, 1) = -dh_param[3].a * (sin(v_ref_joint_angles(2)) * (cos(v_ref_joint_angles(1)) * sin(v_ref_joint_angles(0)) + cos(dh_param[1].alpha) * cos(v_ref_joint_angles(0)) * sin(v_ref_joint_angles(1))) + cos(dh_param[2].alpha) * cos(v_ref_joint_angles(2)) * (sin(v_ref_joint_angles(0)) * sin(v_ref_joint_angles(1)) - cos(dh_param[1].alpha) * cos(v_ref_joint_angles(0)) * cos(v_ref_joint_angles(1))) + sin(dh_param[1].alpha) * sin(dh_param[2].alpha) * cos(v_ref_joint_angles(0)) * cos(v_ref_joint_angles(2))) - dh_param[4].a * (cos(v_ref_joint_angles(3)) * (sin(v_ref_joint_angles(2)) * (cos(v_ref_joint_angles(1)) * sin(v_ref_joint_angles(0)) + cos(dh_param[1].alpha) * cos(v_ref_joint_angles(0)) * sin(v_ref_joint_angles(1))) + cos(dh_param[2].alpha) * cos(v_ref_joint_angles(2)) * (sin(v_ref_joint_angles(0)) * sin(v_ref_joint_angles(1)) - cos(dh_param[1].alpha) * cos(v_ref_joint_angles(0)) * cos(v_ref_joint_angles(1))) + sin(dh_param[1].alpha) * sin(dh_param[2].alpha) * cos(v_ref_joint_angles(0)) * cos(v_ref_joint_angles(2))) - sin(v_ref_joint_angles(3)) * (cos(dh_param[2].alpha) * sin(v_ref_joint_angles(2)) * (sin(v_ref_joint_angles(0)) * sin(v_ref_joint_angles(1)) - cos(dh_param[1].alpha) * cos(v_ref_joint_angles(0)) * cos(v_ref_joint_angles(1))) - cos(v_ref_joint_angles(2)) * (cos(v_ref_joint_angles(1)) * sin(v_ref_joint_angles(0)) + cos(dh_param[1].alpha) * cos(v_ref_joint_angles(0)) * sin(v_ref_joint_angles(1))) + sin(dh_param[1].alpha) * sin(dh_param[2].alpha) * cos(v_ref_joint_angles(0)) * sin(v_ref_joint_angles(2))));
            m_jacobian(1, 2) = -dh_param[4].a * (cos(v_ref_joint_angles(3)) * (sin(v_ref_joint_angles(2)) * (cos(v_ref_joint_angles(1)) * sin(v_ref_joint_angles(0)) + cos(dh_param[1].alpha) * cos(v_ref_joint_angles(0)) * sin(v_ref_joint_angles(1))) + cos(dh_param[2].alpha) * cos(v_ref_joint_angles(2)) * (sin(v_ref_joint_angles(0)) * sin(v_ref_joint_angles(1)) - cos(dh_param[1].alpha) * cos(v_ref_joint_angles(0)) * cos(v_ref_joint_angles(1))) + sin(dh_param[1].alpha) * sin(dh_param[2].alpha) * cos(v_ref_joint_angles(0)) * cos(v_ref_joint_angles(2))) - sin(v_ref_joint_angles(3)) * (cos(dh_param[2].alpha) * sin(v_ref_joint_angles(2)) * (sin(v_ref_joint_angles(0)) * sin(v_ref_joint_angles(1)) - cos(dh_param[1].alpha) * cos(v_ref_joint_angles(0)) * cos(v_ref_joint_angles(1))) - cos(v_ref_joint_angles(2)) * (cos(v_ref_joint_angles(1)) * sin(v_ref_joint_angles(0)) + cos(dh_param[1].alpha) * cos(v_ref_joint_angles(0)) * sin(v_ref_joint_angles(1))) + sin(dh_param[1].alpha) * sin(dh_param[2].alpha) * cos(v_ref_joint_angles(0)) * sin(v_ref_joint_angles(2))));
            m_jacobian(2, 0) = dh_param[4].a * (cos(v_ref_joint_angles(3)) * (sin(dh_param[1].alpha) * cos(v_ref_joint_angles(1)) * cos(v_ref_joint_angles(2)) - cos(dh_param[2].alpha) * sin(dh_param[1].alpha) * sin(v_ref_joint_angles(1)) * sin(v_ref_joint_angles(2))) - sin(v_ref_joint_angles(3)) * (sin(dh_param[1].alpha) * cos(v_ref_joint_angles(1)) * sin(v_ref_joint_angles(2)) + cos(dh_param[2].alpha) * sin(dh_param[1].alpha) * cos(v_ref_joint_angles(2)) * sin(v_ref_joint_angles(1)))) + dh_param[3].a * (sin(dh_param[1].alpha) * cos(v_ref_joint_angles(1)) * cos(v_ref_joint_angles(2)) - cos(dh_param[2].alpha) * sin(dh_param[1].alpha) * sin(v_ref_joint_angles(1)) * sin(v_ref_joint_angles(2))) + dh_param[2].a * sin(dh_param[1].alpha) * cos(v_ref_joint_angles(1));
            m_jacobian(2, 1) = dh_param[4].a * (cos(v_ref_joint_angles(3)) * (cos(dh_param[1].alpha) * sin(dh_param[2].alpha) * cos(v_ref_joint_angles(2)) - sin(dh_param[1].alpha) * sin(v_ref_joint_angles(1)) * sin(v_ref_joint_angles(2)) + cos(dh_param[2].alpha) * sin(dh_param[1].alpha) * cos(v_ref_joint_angles(1)) * cos(v_ref_joint_angles(2))) - sin(v_ref_joint_angles(3)) * (cos(dh_param[1].alpha) * sin(dh_param[2].alpha) * sin(v_ref_joint_angles(2)) + sin(dh_param[1].alpha) * cos(v_ref_joint_angles(2)) * sin(v_ref_joint_angles(1)) + cos(dh_param[2].alpha) * sin(dh_param[1].alpha) * cos(v_ref_joint_angles(1)) * sin(v_ref_joint_angles(2)))) + dh_param[3].a * (cos(dh_param[1].alpha) * sin(dh_param[2].alpha) * cos(v_ref_joint_angles(2)) - sin(dh_param[1].alpha) * sin(v_ref_joint_angles(1)) * sin(v_ref_joint_angles(2)) + cos(dh_param[2].alpha) * sin(dh_param[1].alpha) * cos(v_ref_joint_angles(1)) * cos(v_ref_joint_angles(2)));
            m_jacobian(2, 2) = dh_param[4].a * (cos(v_ref_joint_angles(3)) * (cos(dh_param[1].alpha) * sin(dh_param[2].alpha) * cos(v_ref_joint_angles(2)) - sin(dh_param[1].alpha) * sin(v_ref_joint_angles(1)) * sin(v_ref_joint_angles(2)) + cos(dh_param[2].alpha) * sin(dh_param[1].alpha) * cos(v_ref_joint_angles(1)) * cos(v_ref_joint_angles(2))) - sin(v_ref_joint_angles(3)) * (cos(dh_param[1].alpha) * sin(dh_param[2].alpha) * sin(v_ref_joint_angles(2)) + sin(dh_param[1].alpha) * cos(v_ref_joint_angles(2)) * sin(v_ref_joint_angles(1)) + cos(dh_param[2].alpha) * sin(dh_param[1].alpha) * cos(v_ref_joint_angles(1)) * sin(v_ref_joint_angles(2))));

            m_inv_jacobian = m_jacobian.inverse();

            IK_left_leg_angles.hip_yaw = v_ref_joint_angles(0);
            IK_left_leg_angles.hip_roll = v_ref_joint_angles(1);
            IK_left_leg_angles.hip_pitch = v_ref_joint_angles(2);
            IK_left_leg_angles.knee_pitch = v_ref_joint_angles(3);

            FK_left_foot_pos = computeForwardKinematics(IK_left_leg_angles, leg);
            v_act_foot_pos = {FK_left_foot_pos.x, FK_left_foot_pos.y, FK_left_foot_pos.z};
            v_foot_pos_error = v_ref_foot_pos - v_act_foot_pos;
            v_delta_joint_angles = m_inv_jacobian * v_foot_pos_error;

            IK_epsilon = v_delta_joint_angles.norm();
            v_ref_joint_angles(1) += v_delta_joint_angles(0);
            v_ref_joint_angles(2) += v_delta_joint_angles(1);
            v_ref_joint_angles(3) += v_delta_joint_angles(2);
            IK_iteration_count++;
      }

      IK_left_leg_angles.hip_yaw = 0;//WRAP2_2PI(v_ref_joint_angles(0));
      IK_left_leg_angles.hip_roll = WRAP2_2PI(v_ref_joint_angles(1)) - LEFT_LEG_HIP_ROLL_OFFSET;
      IK_left_leg_angles.hip_pitch = WRAP2_2PI(v_ref_joint_angles(2));
      IK_left_leg_angles.knee_pitch = WRAP2_2PI(v_ref_joint_angles(3));

      return IK_left_leg_angles;
}
