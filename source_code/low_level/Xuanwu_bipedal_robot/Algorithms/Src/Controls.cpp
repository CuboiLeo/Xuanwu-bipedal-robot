#include "Controls.h"

Controls::Controls()
{
}

Direction_Vector_Two Controls::controlCoMPos(const Direction_Vector &ref_com_pos, const Direction_Vector &act_com_pos, const Direction_Vector &ref_left_foot_pos, const Direction_Vector &ref_right_foot_pos)
{
    Direction_Vector_Two ref_foot_pos;
    if (ref_com_pos.x - act_com_pos.x > CoM_adjust_threshold)
    {
        ref_foot_pos.left.x = ref_left_foot_pos.x - CoM_step_size;
        ref_foot_pos.right.x = ref_right_foot_pos.x - CoM_step_size;
    }
    else if (ref_com_pos.x - act_com_pos.x < -CoM_adjust_threshold)
    {
        ref_foot_pos.left.x = ref_left_foot_pos.x + CoM_step_size;
        ref_foot_pos.right.x = ref_right_foot_pos.x + CoM_step_size;
    }
    else
    {
        ref_foot_pos.left.x = ref_left_foot_pos.x;
        ref_foot_pos.right.x = ref_right_foot_pos.x;
    }

    if (ref_com_pos.y - act_com_pos.y > CoM_adjust_threshold)
    {
        ref_foot_pos.left.y = ref_left_foot_pos.y - CoM_step_size;
        ref_foot_pos.right.y = ref_right_foot_pos.y - CoM_step_size;
    }
    else if (ref_com_pos.y - act_com_pos.y < -CoM_adjust_threshold)
    {
        ref_foot_pos.left.y = ref_left_foot_pos.y + CoM_step_size;
        ref_foot_pos.right.y = ref_right_foot_pos.y + CoM_step_size;
    }
    else
    {
        ref_foot_pos.left.y = ref_left_foot_pos.y;
        ref_foot_pos.right.y = ref_right_foot_pos.y;
    }

    ref_foot_pos.left.z = ref_left_foot_pos.z;
    ref_foot_pos.right.z = ref_right_foot_pos.z;

    return ref_foot_pos;
}
