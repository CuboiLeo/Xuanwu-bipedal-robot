#ifndef FORWARD_KINEMATICS_H
#define FORWARD_KINEMATICS_H

#ifdef __cplusplus
extern "C" {
#endif

#include "math.h"
#include "stdint.h"
#include "Robot_Types.h"
#include "Robot.h"
	
class Kinematics {
    public:
        Kinematics();
        void computeForwardKinematics(Robot* robot);
        void computeInverseKinematics(Robot* robot);
    private:
        Joint_Angle IK_left_leg_angles1;
        Joint_Angle IK_left_leg_angles2;
        Joint_Angle IK_left_leg_angles_final;
        Joint_Angle IK_right_leg_angles1;
        Joint_Angle IK_right_leg_angles2;
        Joint_Angle IK_right_leg_angles_final;

        Foot_Position FK_left_foot_pos;
        Foot_Position FK_right_foot_pos;    
};

#ifdef __cplusplus
}
#endif

#endif
