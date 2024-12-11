#ifndef CONTROLS_H
#define CONTROLS_H

#include "robot_types.h"
#include "Eigen/Dense"
#include "Fusion.h"

class Controls
{
public:
    Direction_Vector controlCoMPos(const Direction_Vector &ref_CoM_pos, const Direction_Vector &act_CoM_pos, const Joint_Angles &act_left_angles, const Joint_Angles &act_right_angles, const FusionMatrix &rotation_matrix);

private:
    static constexpr uint8_t MAX_ITERATIONS = 100;
    static constexpr float ERROR_TOLERANCE = 0.005f;
    static constexpr float ERROR_GAIN = 0.3f;
};

#endif
