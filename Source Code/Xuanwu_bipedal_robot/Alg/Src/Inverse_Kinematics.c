#include "Inverse_Kinematics.h"

IK_Solution_t g_IK_Solution;
void Inverse_Kinematics_Reduced_Order_Analytic(void)
{
    g_IK_Solution.left_leg_1.hip_yaw = atan2f(g_Robot.ref_left_foot.y,g_Robot.ref_left_foot.x)-atan2f(DH_A2,sqrtf(g_Robot.ref_left_foot.x*g_Robot.ref_left_foot.x
                                     +g_Robot.ref_left_foot.y*g_Robot.ref_left_foot.y-DH_A2*DH_A2));
    g_IK_Solution.left_leg_2.hip_yaw = PI+atan2f(g_Robot.ref_left_foot.y,g_Robot.ref_left_foot.x)+atan2f(-sqrtf(g_Robot.ref_left_foot.x*g_Robot.ref_left_foot.x
                                     +g_Robot.ref_left_foot.y*g_Robot.ref_left_foot.y-DH_A2*DH_A2),DH_A2);

    g_IK_Solution.left_leg_1.knee_pitch = atan2f(sqrtf(1.0f-((g_Robot.ref_left_foot.x*g_Robot.ref_left_foot.x+g_Robot.ref_left_foot.y*g_Robot.ref_left_foot.y
                                        +g_Robot.ref_left_foot.z*g_Robot.ref_left_foot.z-DH_A2*DH_A2-DH_A3*DH_A3-DH_A4*DH_A4)/(2*DH_A3*DH_A4))
                                        *((g_Robot.ref_left_foot.x*g_Robot.ref_left_foot.x+g_Robot.ref_left_foot.y*g_Robot.ref_left_foot.y
                                        +g_Robot.ref_left_foot.z*g_Robot.ref_left_foot.z-DH_A2*DH_A2-DH_A3*DH_A3-DH_A4*DH_A4)/(2*DH_A3*DH_A4)))
                                        ,((g_Robot.ref_left_foot.x*g_Robot.ref_left_foot.x+g_Robot.ref_left_foot.y*g_Robot.ref_left_foot.y+g_Robot.ref_left_foot.z
                                        *g_Robot.ref_left_foot.z-DH_A2*DH_A2-DH_A3*DH_A3-DH_A4*DH_A4)/(2*DH_A3*DH_A4)));
    g_IK_Solution.left_leg_2.knee_pitch = atan2f(-sqrtf(1.0f-((g_Robot.ref_left_foot.x*g_Robot.ref_left_foot.x+g_Robot.ref_left_foot.y*g_Robot.ref_left_foot.y
                                        +g_Robot.ref_left_foot.z*g_Robot.ref_left_foot.z-DH_A2*DH_A2-DH_A3*DH_A3-DH_A4*DH_A4)/(2*DH_A3*DH_A4))
                                        *((g_Robot.ref_left_foot.x*g_Robot.ref_left_foot.x+g_Robot.ref_left_foot.y*g_Robot.ref_left_foot.y
                                        +g_Robot.ref_left_foot.z*g_Robot.ref_left_foot.z-DH_A2*DH_A2-DH_A3*DH_A3-DH_A4*DH_A4)/(2*DH_A3*DH_A4)))
                                        ,((g_Robot.ref_left_foot.x*g_Robot.ref_left_foot.x+g_Robot.ref_left_foot.y*g_Robot.ref_left_foot.y+g_Robot.ref_left_foot.z
                                        *g_Robot.ref_left_foot.z-DH_A2*DH_A2-DH_A3*DH_A3-DH_A4*DH_A4)/(2*DH_A3*DH_A4)));

    g_IK_Solution.left_leg_1.hip_pitch = atan2f(g_Robot.ref_left_foot.z,sqrtf(g_Robot.ref_left_foot.x*g_Robot.ref_left_foot.x+g_Robot.ref_left_foot.y
                                       *g_Robot.ref_left_foot.y-DH_A2*DH_A2))-atan2f(DH_A4*sin(g_IK_Solution.left_leg_1.knee_pitch),DH_A3+DH_A4
                                       *cos(g_IK_Solution.left_leg_1.knee_pitch));
    g_IK_Solution.left_leg_2.hip_pitch = atan2f(g_Robot.ref_left_foot.z,sqrtf(g_Robot.ref_left_foot.x*g_Robot.ref_left_foot.x+g_Robot.ref_left_foot.y
                                       *g_Robot.ref_left_foot.y-DH_A2*DH_A2))-atan2f(DH_A4*sin(g_IK_Solution.left_leg_2.knee_pitch),DH_A3+DH_A4
                                       *cos(g_IK_Solution.left_leg_2.knee_pitch));
}