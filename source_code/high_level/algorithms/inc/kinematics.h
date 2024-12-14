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
      Direction_Vector computeCenterOfMass(const Joint_Angles &joint_angles_left, const Joint_Angles &joint_angles_right, const FusionMatrix &rotation_matrix);
      Direction_Vector computeFootFK(const Joint_Angles &joint_angles, const uint8_t &leg_id);
      Joint_Angles computeFootIK(const Direction_Vector &act_foot_pos, const Direction_Vector &ref_foot_pos, const Joint_Angles &act_joint_angles, const uint8_t &leg_id);
      Direction_Vector computeCenterFK(const Joint_Angles &joint_angles, const uint8_t &leg_id);
      Joint_Angles computeCenterIK(const Direction_Vector &act_center_pos, const Direction_Vector &ref_center_pos, const Joint_Angles &act_joint_angles, const uint8_t &leg_id);

private:
      static constexpr uint8_t MAX_ITERATIONS = 50;    // Maximum number of iterations for the inverse kinematics
      static constexpr float ERROR_TOLERANCE = 0.005f; // Error tolerance for the inverse kinematics
      float IK_LAMBDA = 0.001f;                        // Damping factor for the inverse kinematics - SR Inverse Levenberg-Marquardt method

      Direction_Vector CoM_link1 = {}; // center of mass of the main control assembly + 2 * hip yaw connector + 2 * hip yaw motors
      Direction_Vector CoM_link2 = {}; // center of mass of the left hip roll motor + hip roll connector
      Direction_Vector CoM_link3 = {}; // center of mass of the left hip pitch motor + hip pitch connector
      Direction_Vector CoM_link4 = {}; // center of mass of the left knee pitch motor + knee pitch connector
      Direction_Vector CoM_link5 = {}; // center of mass of the left ankle connector + foot connector
      Direction_Vector CoM_link6 = {}; // center of mass of the right hip roll motor + hip roll connector
      Direction_Vector CoM_link7 = {}; // center of mass of the right hip pitch motor + hip pitch connector
      Direction_Vector CoM_link8 = {}; // center of mass of the right knee pitch motor + knee pitch connector
      Direction_Vector CoM_link9 = {}; // center of mass of the right ankle connector + foot connector
      Direction_Vector CoM_body_frame = {};        // center of mass of the robot in the body frame
};

#endif
