#include "controls.h"
#include "kinematics.h"
#include "dynamics.h"

Kinematics kinematics;
Dynamics dynamics;

Direction_Vector Controls::controlCoMPos(const Direction_Vector &ref_CoM_pos, const Direction_Vector &act_CoM_pos, const Joint_Angles &act_left_angles, const Joint_Angles &act_right_angles, const FusionMatrix &rotation_matrix)
{
    // Compute the CoM position error
    Eigen::Vector3f v_ref_CoM_pos(ref_CoM_pos.x, ref_CoM_pos.y, ref_CoM_pos.z);
    Eigen::Vector3f v_act_CoM_pos(act_CoM_pos.x, act_CoM_pos.y, act_CoM_pos.z);
    Eigen::Vector3f v_CoM_pos_error = v_ref_CoM_pos - v_act_CoM_pos;
    float CoM_pos_error = v_CoM_pos_error.head(2).norm();

    // Setup the initial conditions
    Direction_Vector computed_ref_center_pos = {0, 0, -0.05f};
    Joint_Angles computed_left_angles = act_left_angles;
    Joint_Angles computed_right_angles = act_right_angles;

    uint8_t iteration_count = 0;

    // Iteratively compute the CoM position and adjust the center position to minimize the error
    while (CoM_pos_error > ERROR_TOLERANCE && iteration_count < MAX_ITERATIONS)
    {
        Direction_Vector computed_left_act_center_pos = kinematics.computeCenterFK(computed_left_angles, LEFT_LEG_ID);
        Direction_Vector computed_right_act_center_pos = kinematics.computeCenterFK(computed_right_angles, RIGHT_LEG_ID);
        Joint_Angles computed_left_angles = kinematics.computeCenterIK(computed_left_act_center_pos, computed_ref_center_pos, computed_left_angles, LEFT_LEG_ID);
        Joint_Angles computed_right_angles = kinematics.computeCenterIK(computed_right_act_center_pos, computed_ref_center_pos, computed_right_angles, RIGHT_LEG_ID);

        Direction_Vector computed_CoM_pos = dynamics.computeCenterOfMass(computed_left_angles, computed_right_angles, rotation_matrix);
        Eigen::Vector3f v_computed_CoM_pos(computed_CoM_pos.x, computed_CoM_pos.y, computed_CoM_pos.z);
        v_CoM_pos_error = v_ref_CoM_pos - v_computed_CoM_pos;
        CoM_pos_error = v_CoM_pos_error.head(2).norm();

        if (CoM_pos_error > ERROR_TOLERANCE)
        {
            computed_ref_center_pos.x += ERROR_GAIN * v_CoM_pos_error(0);
            computed_ref_center_pos.y += ERROR_GAIN * v_CoM_pos_error(1);
        }
        
        iteration_count++;
    }

    return computed_ref_center_pos;
}
