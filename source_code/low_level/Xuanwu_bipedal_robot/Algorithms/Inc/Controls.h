#ifndef CONTROLS_H
#define CONTROLS_H

#include "Robot_Types.h"

class Controls
{
public:
    Controls();
    Direction_Vector_Two controlCoMPos(const Direction_Vector &ref_com_pos, const Direction_Vector &act_com_pos, const Direction_Vector &ref_left_foot_pos, const Direction_Vector &ref_right_foot_pos);

private:
    static constexpr float CoM_adjust_threshold = 0.002f;
    static constexpr float CoM_step_size = 0.00004f;
};

#endif
