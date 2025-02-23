#ifndef WALKING_PATTERNS_H
#define WALKING_PATTERNS_H

#include <stdint.h>
#include "robot_types.h"
#include "robot_configs.h"

class Walking_Patterns
{
    public:
    Position computeLIPM(const Position &act_CoM_pos, const Velocity &act_CoM_vel);

};
    

#endif