#ifndef INVERSE_KINEMATICS_H
#define INVERSE_KINEMATICS_H

#include "stdint.h"
#include "Robot.h"

typedef struct IK_Solution
{
    Joint_Angle_t left_leg_1;
    Joint_Angle_t left_leg_2;

    Joint_Angle_t right_leg_1;
    Joint_Angle_t right_leg_2;
}IK_Solution_t;

extern IK_Solution_t g_IK_Solution;
void Inverse_Kinematics_Reduced_Order_Analytic(void);

#endif
