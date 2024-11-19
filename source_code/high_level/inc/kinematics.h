#ifndef FORWARD_KINEMATICS_H
#define FORWARD_KINEMATICS_H

#include "stdint.h"
#include "robot_types.h"

class Kinematics
{
public:
      static constexpr uint8_t MAX_ITERATIONS = 50;
      static constexpr float UPDATE_TOLERANCE = 0.005f;
      static constexpr float ERROR_TOLERANCE = 0.005f;
      static constexpr float MAX_X = 0.2299f;
      static constexpr float MAX_Y = 0.0001f;
      static constexpr float MAX_Z = 0.6399f;
      static constexpr float STEP_SIZE = 0.15f;
      static constexpr float STEP_HEIGHT = 0.06f;
      static constexpr float SINGLE_STANCE_PERIOD = 0.4f;
      static constexpr float DOUBLE_STANCE_PERIOD = 0.3f;

      Kinematics();
      Direction_Vector computeForwardKinematics(Joint_Angle joint_angles, const int leg);
      Joint_Angle computeInverseKinematics(const Direction_Vector &ref_foot_pos, const Direction_Vector &act_foot_pos, const Joint_Angle &joint_angles, const int leg);
      Direction_Vector generateTrajectory(const Direction_Vector &ref_robot_vel, const int leg);

private:
      Joint_Angle IK_left_leg_angles;
      Joint_Angle IK_right_leg_angles;

      Direction_Vector FK_left_foot_pos;
      Direction_Vector FK_right_foot_pos;

      uint8_t IK_iteration_count = 0;
      float IK_epsilon = 1.0f;

      float x_deviation = 0.0f;
      float y_deviation = 0.0f;
      float z_deviation = 0.0f;
      float left_foot_phase = 0.0f;
      float right_foot_phase = 0.0f;
      float double_stance_phase = 0.0f;
      bool left_foot_swing_flag = false;
      bool right_foot_swing_flag = false;
      bool double_stance_flag = false;
};

/* Forward kinematics computation, the final transformation matrix is computed in MATLAB
T05 = T01*T12*T23*T34*T45

T01 = [cos(theta1), -sin(theta1), 0,  a]
      [sin(theta1),  cos(theta1), 0,  0]
      [          0,            0, 1,  0]
      [          0,            0, 0,  1]
T12 = [            cos(theta2),            -sin(theta2),            0, dh_param[1].a]
      [cos(dh_param[1].alpha)*sin(theta2), cos(dh_param[1].alpha)*cos(theta2), -sin(dh_param[1].alpha),  0]
      [sin(dh_param[1].alpha)*sin(theta2), sin(dh_param[1].alpha)*cos(theta2),  cos(dh_param[1].alpha),  0]
      [                      0,                       0,            0,  1]
T23 = [            cos(theta3),            -sin(theta3),            0, dh_param[2].a]
      [cos(dh_param[2].alpha)*sin(theta3), cos(dh_param[2].alpha)*cos(theta3), -sin(dh_param[2].alpha),  0]
      [sin(dh_param[2].alpha)*sin(theta3), sin(dh_param[2].alpha)*cos(theta3),  cos(dh_param[2].alpha),  0]
      [                      0,                       0,            0,  1]
T34 = [cos(theta4), -sin(theta4), 0, dh_param[3].a]
      [sin(theta4),  cos(theta4), 0,  0]
      [          0,            0, 1,  0]
      [          0,            0, 0,  1]
T45 = [1, 0, 0, dh_param[4].a]
      [0, 1, 0,  0]
      [0, 0, 1,  0]
      [0, 0, 0,  1]
*/
#endif
