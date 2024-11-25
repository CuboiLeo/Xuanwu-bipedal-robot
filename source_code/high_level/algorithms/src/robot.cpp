#include "robot.h"

void Robot::setMotorData(Motor &motor)
{
    motor.setRefPos(Left_Hip_Yaw, leg_angles.left.ref.hip_yaw);
    motor.setRefPos(Left_Hip_Roll, leg_angles.left.ref.hip_roll);
    motor.setRefPos(Left_Hip_Pitch, leg_angles.left.ref.hip_pitch);
    motor.setRefPos(Left_Knee_Pitch, leg_angles.left.ref.knee_pitch);
    motor.setRefPos(Right_Hip_Yaw, leg_angles.right.ref.hip_yaw);
    motor.setRefPos(Right_Hip_Roll, leg_angles.right.ref.hip_roll);
    motor.setRefPos(Right_Hip_Pitch, leg_angles.right.ref.hip_pitch);
    motor.setRefPos(Right_Knee_Pitch, leg_angles.right.ref.knee_pitch);
}