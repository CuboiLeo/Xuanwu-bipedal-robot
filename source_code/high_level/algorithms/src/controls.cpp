#include "controls.h"
#include "kinematics.h"
#include "dynamics.h"

Kinematics kinematics;
Dynamics dynamics;

Direction_Vector Controls::controlCoMPos(const Direction_Vector &ref_CoM_pos, const Direction_Vector &act_CoM_pos, const Joint_Angles &act_left_angles, const Joint_Angles &act_right_angles, const FusionMatrix &rotation_matrix)
{
    return {0.0f, 0.0f, 0.0f};
}
