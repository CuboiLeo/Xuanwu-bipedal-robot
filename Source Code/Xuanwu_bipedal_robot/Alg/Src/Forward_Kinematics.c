#include "Forward_Kinematics.h"
#include "User_Math.h"

void Forward_Kinematics_DH(void)
{
    g_Robot.left_foot.x = DH_A1 + DH_A4*(cosf(g_Robot.left_leg.hip_pitch)*(cosf(g_Robot.left_leg.hip_yaw)*cosf(g_Robot.left_leg.hip_roll+PI/2) 
                        - cosf(DH_ALPHA2)*sinf(g_Robot.left_leg.hip_yaw)*sinf(g_Robot.left_leg.hip_roll+PI/2)) - cosf(DH_ALPHA3)*sinf(g_Robot.left_leg.hip_pitch)
                        *(cosf(g_Robot.left_leg.hip_yaw)*sinf(g_Robot.left_leg.hip_roll+PI/2) + cosf(DH_ALPHA2)*cosf(g_Robot.left_leg.hip_roll+PI/2)
                        *sinf(g_Robot.left_leg.hip_yaw)) + sinf(DH_ALPHA2)*sinf(DH_ALPHA3)*sinf(g_Robot.left_leg.hip_yaw)*sinf(g_Robot.left_leg.hip_pitch)) 
                        + DH_A5*(cosf(g_Robot.left_leg.knee_pitch)*(cosf(g_Robot.left_leg.hip_pitch)*(cosf(g_Robot.left_leg.hip_yaw)*cosf(g_Robot.left_leg.hip_roll+PI/2)
                        - cosf(DH_ALPHA2)*sinf(g_Robot.left_leg.hip_yaw)*sinf(g_Robot.left_leg.hip_roll+PI/2)) - cosf(DH_ALPHA3)*sinf(g_Robot.left_leg.hip_pitch)
                        *(cosf(g_Robot.left_leg.hip_yaw)*sinf(g_Robot.left_leg.hip_roll+PI/2) + cosf(DH_ALPHA2)*cosf(g_Robot.left_leg.hip_roll+PI/2)
                        *sinf(g_Robot.left_leg.hip_yaw)) + sinf(DH_ALPHA2)*sinf(DH_ALPHA3)*sinf(g_Robot.left_leg.hip_yaw)*sinf(g_Robot.left_leg.hip_pitch)) 
                        - sinf(g_Robot.left_leg.knee_pitch)*(sinf(g_Robot.left_leg.hip_pitch)*(cosf(g_Robot.left_leg.hip_yaw)*cosf(g_Robot.left_leg.hip_roll+PI/2) 
                        - cosf(DH_ALPHA2)*sinf(g_Robot.left_leg.hip_yaw)*sinf(g_Robot.left_leg.hip_roll+PI/2)) + cosf(DH_ALPHA3)*cosf(g_Robot.left_leg.hip_pitch)
                        *(cosf(g_Robot.left_leg.hip_yaw)*sinf(g_Robot.left_leg.hip_roll+PI/2) + cosf(DH_ALPHA2)*cosf(g_Robot.left_leg.hip_roll+PI/2)
                        *sinf(g_Robot.left_leg.hip_yaw)) - sinf(DH_ALPHA2)*sinf(DH_ALPHA3)*cosf(g_Robot.left_leg.hip_pitch)*sinf(g_Robot.left_leg.hip_yaw))) 
                        + DH_A3*(cosf(g_Robot.left_leg.hip_yaw)*cosf(g_Robot.left_leg.hip_roll+PI/2) - cosf(DH_ALPHA2)*sinf(g_Robot.left_leg.hip_yaw)
                        *sinf(g_Robot.left_leg.hip_roll+PI/2)) + DH_A2*cosf(g_Robot.left_leg.hip_yaw);
    g_Robot.left_foot.y = DH_A3*(cosf(g_Robot.left_leg.hip_roll+PI/2)*sinf(g_Robot.left_leg.hip_yaw) + cosf(DH_ALPHA2)*cosf(g_Robot.left_leg.hip_yaw)
                        *sinf(g_Robot.left_leg.hip_roll+PI/2)) + DH_A2*sinf(g_Robot.left_leg.hip_yaw) - DH_A5*(sinf(g_Robot.left_leg.knee_pitch)
                        *(sinf(g_Robot.left_leg.hip_pitch)*(cosf(g_Robot.left_leg.hip_roll+PI/2)*sinf(g_Robot.left_leg.hip_yaw) + cosf(DH_ALPHA2)
                        *cosf(g_Robot.left_leg.hip_yaw)*sinf(g_Robot.left_leg.hip_roll+PI/2)) + cosf(DH_ALPHA3)*cosf(g_Robot.left_leg.hip_pitch)
                        *(sinf(g_Robot.left_leg.hip_yaw)*sinf(g_Robot.left_leg.hip_roll+PI/2) - cosf(DH_ALPHA2)*cosf(g_Robot.left_leg.hip_yaw)
                        *cosf(g_Robot.left_leg.hip_roll+PI/2)) + sinf(DH_ALPHA2)*sinf(DH_ALPHA3)*cosf(g_Robot.left_leg.hip_yaw)*cosf(g_Robot.left_leg.hip_pitch)) 
                        + cosf(g_Robot.left_leg.knee_pitch)*(cosf(DH_ALPHA3)*sinf(g_Robot.left_leg.hip_pitch)*(sinf(g_Robot.left_leg.hip_yaw)
                        *sinf(g_Robot.left_leg.hip_roll+PI/2) - cosf(DH_ALPHA2)*cosf(g_Robot.left_leg.hip_yaw)*cosf(g_Robot.left_leg.hip_roll+PI/2)) 
                        - cosf(g_Robot.left_leg.hip_pitch)*(cosf(g_Robot.left_leg.hip_roll+PI/2)*sinf(g_Robot.left_leg.hip_yaw) + cosf(DH_ALPHA2)
                        *cosf(g_Robot.left_leg.hip_yaw)*sinf(g_Robot.left_leg.hip_roll+PI/2)) + sinf(DH_ALPHA2)*sinf(DH_ALPHA3)*cosf(g_Robot.left_leg.hip_yaw)
                        *sinf(g_Robot.left_leg.hip_pitch))) - DH_A4*(cosf(DH_ALPHA3)*sinf(g_Robot.left_leg.hip_pitch)*(sinf(g_Robot.left_leg.hip_yaw)
                        *sinf(g_Robot.left_leg.hip_roll+PI/2) - cosf(DH_ALPHA2)*cosf(g_Robot.left_leg.hip_yaw)*cosf(g_Robot.left_leg.hip_roll+PI/2)) 
                        - cosf(g_Robot.left_leg.hip_pitch)*(cosf(g_Robot.left_leg.hip_roll+PI/2)*sinf(g_Robot.left_leg.hip_yaw) + cosf(DH_ALPHA2)
                        *cosf(g_Robot.left_leg.hip_yaw)*sinf(g_Robot.left_leg.hip_roll+PI/2)) + sinf(DH_ALPHA2)*sinf(DH_ALPHA3)*cosf(g_Robot.left_leg.hip_yaw)
                        *sinf(g_Robot.left_leg.hip_pitch));
    g_Robot.left_foot.z = DH_A5*(cosf(g_Robot.left_leg.knee_pitch)*(cosf(DH_ALPHA2)*sinf(DH_ALPHA3)*sinf(g_Robot.left_leg.hip_pitch) + sinf(DH_ALPHA2)
                        *cosf(g_Robot.left_leg.hip_pitch)*sinf(g_Robot.left_leg.hip_roll+PI/2) + cosf(DH_ALPHA3)*sinf(DH_ALPHA2)*cosf(g_Robot.left_leg.hip_roll+PI/2)
                        *sinf(g_Robot.left_leg.hip_pitch)) + sinf(g_Robot.left_leg.knee_pitch)*(cosf(DH_ALPHA2)*sinf(DH_ALPHA3)*cosf(g_Robot.left_leg.hip_pitch) 
                        - sinf(DH_ALPHA2)*sinf(g_Robot.left_leg.hip_roll+PI/2)*sinf(g_Robot.left_leg.hip_pitch) + cosf(DH_ALPHA3)*sinf(DH_ALPHA2)*cosf(g_Robot.left_leg.hip_roll+PI/2)
                        *cosf(g_Robot.left_leg.hip_pitch))) + DH_A4*(cosf(DH_ALPHA2)*sinf(DH_ALPHA3)*sinf(g_Robot.left_leg.hip_pitch) + sinf(DH_ALPHA2)
                        *cosf(g_Robot.left_leg.hip_pitch)*sinf(g_Robot.left_leg.hip_roll+PI/2) + cosf(DH_ALPHA3)*sinf(DH_ALPHA2)*cosf(g_Robot.left_leg.hip_roll+PI/2)
                        *sinf(g_Robot.left_leg.hip_pitch)) + DH_A3*sinf(DH_ALPHA2)*sinf(g_Robot.left_leg.hip_roll+PI/2);

    g_Robot.right_foot.x = DH_A6 + DH_A9*(cosf(g_Robot.right_leg.hip_pitch)*(cosf(g_Robot.right_leg.hip_yaw)*cosf(g_Robot.right_leg.hip_roll-PI/2)
                         - cosf(DH_ALPHA7)*sinf(g_Robot.right_leg.hip_yaw)*sinf(g_Robot.right_leg.hip_roll-PI/2)) - cosf(DH_ALPHA8)*sinf(g_Robot.right_leg.hip_pitch)
                         *(cosf(g_Robot.right_leg.hip_yaw)*sinf(g_Robot.right_leg.hip_roll-PI/2) + cosf(DH_ALPHA7)*cosf(g_Robot.right_leg.hip_roll-PI/2)
                         *sinf(g_Robot.right_leg.hip_yaw)) + sinf(DH_ALPHA7)*sinf(DH_ALPHA8)*sinf(g_Robot.right_leg.hip_yaw)*sinf(g_Robot.right_leg.hip_pitch)) 
                         + DH_A10*(cosf(g_Robot.right_leg.knee_pitch)*(cosf(g_Robot.right_leg.hip_pitch)*(cosf(g_Robot.right_leg.hip_yaw)*cosf(g_Robot.right_leg.hip_roll-PI/2)
                         - cosf(DH_ALPHA7)*sinf(g_Robot.right_leg.hip_yaw)*sinf(g_Robot.right_leg.hip_roll-PI/2)) - cosf(DH_ALPHA8)*sinf(g_Robot.right_leg.hip_pitch)
                         *(cosf(g_Robot.right_leg.hip_yaw)*sinf(g_Robot.right_leg.hip_roll-PI/2) + cosf(DH_ALPHA7)*cosf(g_Robot.right_leg.hip_roll-PI/2)
                         *sinf(g_Robot.right_leg.hip_yaw)) + sinf(DH_ALPHA7)*sinf(DH_ALPHA8)*sinf(g_Robot.right_leg.hip_yaw)*sinf(g_Robot.right_leg.hip_pitch)) 
                         - sinf(g_Robot.right_leg.knee_pitch)*(sinf(g_Robot.right_leg.hip_pitch)*(cosf(g_Robot.right_leg.hip_yaw)*cosf(g_Robot.right_leg.hip_roll-PI/2) 
                         - cosf(DH_ALPHA7)*sinf(g_Robot.right_leg.hip_yaw)*sinf(g_Robot.right_leg.hip_roll-PI/2)) + cosf(DH_ALPHA8)*cosf(g_Robot.right_leg.hip_pitch)
                         *(cosf(g_Robot.right_leg.hip_yaw)*sinf(g_Robot.right_leg.hip_roll-PI/2) + cosf(DH_ALPHA7)*cosf(g_Robot.right_leg.hip_roll-PI/2)
                         *sinf(g_Robot.right_leg.hip_yaw)) - sinf(DH_ALPHA7)*sinf(DH_ALPHA8)*cosf(g_Robot.right_leg.hip_pitch)*sinf(g_Robot.right_leg.hip_yaw)))
                         + DH_A8*(cosf(g_Robot.right_leg.hip_yaw)*cosf(g_Robot.right_leg.hip_roll-PI/2) - cosf(DH_ALPHA7)*sinf(g_Robot.right_leg.hip_yaw)
                         *sinf(g_Robot.right_leg.hip_roll-PI/2)) + DH_A7*cosf(g_Robot.right_leg.hip_yaw);
    g_Robot.right_foot.y = DH_A8*(cosf(g_Robot.right_leg.hip_roll-PI/2)*sinf(g_Robot.right_leg.hip_yaw) + cosf(DH_ALPHA7)*cosf(g_Robot.right_leg.hip_yaw)
                         *sinf(g_Robot.right_leg.hip_roll-PI/2)) + DH_A7*sinf(g_Robot.right_leg.hip_yaw) - DH_A10*(sinf(g_Robot.right_leg.knee_pitch)
                         *(sinf(g_Robot.right_leg.hip_pitch)*(cosf(g_Robot.right_leg.hip_roll-PI/2)*sinf(g_Robot.right_leg.hip_yaw) + cosf(DH_ALPHA7)
                         *cosf(g_Robot.right_leg.hip_yaw)*sinf(g_Robot.right_leg.hip_roll-PI/2)) + cosf(DH_ALPHA8)*cosf(g_Robot.right_leg.hip_pitch)
                         *(sinf(g_Robot.right_leg.hip_yaw)*sinf(g_Robot.right_leg.hip_roll-PI/2) - cosf(DH_ALPHA7)*cosf(g_Robot.right_leg.hip_yaw)
                         *cosf(g_Robot.right_leg.hip_roll-PI/2)) + sinf(DH_ALPHA7)*sinf(DH_ALPHA8)*cosf(g_Robot.right_leg.hip_yaw)*cosf(g_Robot.right_leg.hip_pitch)) 
                         + cosf(g_Robot.right_leg.knee_pitch)*(cosf(DH_ALPHA8)*sinf(g_Robot.right_leg.hip_pitch)*(sinf(g_Robot.right_leg.hip_yaw)
                         *sinf(g_Robot.right_leg.hip_roll-PI/2) - cosf(DH_ALPHA7)*cosf(g_Robot.right_leg.hip_yaw)*cosf(g_Robot.right_leg.hip_roll-PI/2)) 
                         - cosf(g_Robot.right_leg.hip_pitch)*(cosf(g_Robot.right_leg.hip_roll-PI/2)*sinf(g_Robot.right_leg.hip_yaw) + cosf(DH_ALPHA7)
                         *cosf(g_Robot.right_leg.hip_yaw)*sinf(g_Robot.right_leg.hip_roll-PI/2)) + sinf(DH_ALPHA7)*sinf(DH_ALPHA8)*cosf(g_Robot.right_leg.hip_yaw)
                         *sinf(g_Robot.right_leg.hip_pitch))) - DH_A9*(cosf(DH_ALPHA8)*sinf(g_Robot.right_leg.hip_pitch)*(sinf(g_Robot.right_leg.hip_yaw)
                         *sinf(g_Robot.right_leg.hip_roll-PI/2) - cosf(DH_ALPHA7)*cosf(g_Robot.right_leg.hip_yaw)*cosf(g_Robot.right_leg.hip_roll-PI/2))
                         - cosf(g_Robot.right_leg.hip_pitch)*(cosf(g_Robot.right_leg.hip_roll-PI/2)*sinf(g_Robot.right_leg.hip_yaw) + cosf(DH_ALPHA7)
                         *cosf(g_Robot.right_leg.hip_yaw)*sinf(g_Robot.right_leg.hip_roll-PI/2)) + sinf(DH_ALPHA7)*sinf(DH_ALPHA8)*cosf(g_Robot.right_leg.hip_yaw)
                         *sinf(g_Robot.right_leg.hip_pitch));
    g_Robot.right_foot.z = DH_A10*(cosf(g_Robot.right_leg.knee_pitch)*(cosf(DH_ALPHA7)*sinf(DH_ALPHA8)*sinf(g_Robot.right_leg.hip_pitch) + sinf(DH_ALPHA7)
                         *cosf(g_Robot.right_leg.hip_pitch)*sinf(g_Robot.right_leg.hip_roll-PI/2) + cosf(DH_ALPHA8)*sinf(DH_ALPHA7)*cosf(g_Robot.right_leg.hip_roll-PI/2)
                         *sinf(g_Robot.right_leg.hip_pitch)) + sinf(g_Robot.right_leg.knee_pitch)*(cosf(DH_ALPHA7)*sinf(DH_ALPHA8)*cosf(g_Robot.right_leg.hip_pitch) 
                         - sinf(DH_ALPHA7)*sinf(g_Robot.right_leg.hip_roll-PI/2)*sinf(g_Robot.right_leg.hip_pitch) + cosf(DH_ALPHA8)*sinf(DH_ALPHA7)*cosf(g_Robot.right_leg.hip_roll-PI/2)
                         *cosf(g_Robot.right_leg.hip_pitch))) + DH_A9*(cosf(DH_ALPHA7)*sinf(DH_ALPHA8)*sinf(g_Robot.right_leg.hip_pitch) + sinf(DH_ALPHA7)
                         *cosf(g_Robot.right_leg.hip_pitch)*sinf(g_Robot.right_leg.hip_roll-PI/2) + cosf(DH_ALPHA8)*sinf(DH_ALPHA7)*cosf(g_Robot.right_leg.hip_roll-PI/2)
                         *sinf(g_Robot.right_leg.hip_pitch)) + DH_A8*sinf(DH_ALPHA7)*sinf(g_Robot.right_leg.hip_roll-PI/2);

}

/*
T05 = T01*T12*T23*T34*T45

T01 = [cos(theta1), -sin(theta1), 0, DH_A1]
      [sin(theta1),  cos(theta1), 0,  0]
      [          0,            0, 1,  0]
      [          0,            0, 0,  1]
T12 = [            cos(theta2),            -sin(theta2),            0, DH_A2]
      [cos(DH_ALPHA2)*sin(theta2), cos(DH_ALPHA2)*cos(theta2), -sin(DH_ALPHA2),  0]
      [sin(DH_ALPHA2)*sin(theta2), sin(DH_ALPHA2)*cos(theta2),  cos(DH_ALPHA2),  0]
      [                      0,                       0,            0,  1]
T23 = [            cos(theta3),            -sin(theta3),            0, DH_A3]
      [cos(DH_ALPHA3)*sin(theta3), cos(DH_ALPHA3)*cos(theta3), -sin(DH_ALPHA3),  0]
      [sin(DH_ALPHA3)*sin(theta3), sin(DH_ALPHA3)*cos(theta3),  cos(DH_ALPHA3),  0]
      [                      0,                       0,            0,  1]
T34 = [cos(theta4), -sin(theta4), 0, DH_A4]
      [sin(theta4),  cos(theta4), 0,  0]
      [          0,            0, 1,  0]
      [          0,            0, 0,  1]
T45 = [1, 0, 0, DH_A5]
      [0, 1, 0,  0]
      [0, 0, 1,  0]
      [0, 0, 0,  1]
*/