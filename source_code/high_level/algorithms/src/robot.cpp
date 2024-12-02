#include "robot.h"

Robot::Robot()
{
    leg_angles.left.act = {0.0f, 0.0f, 0.0f, 0.0f};
    leg_angles.right.act = {0.0f, 0.0f, 0.0f, 0.0f};
    leg_angles.left.ref = {0.0f, 0.0f, 0.0f, 0.0f};
    leg_angles.right.ref = {0.0f, 0.0f, 0.0f, 0.0f};
    foot_pos.left.act = {0.0f, 0.0f, 0.0f};
    foot_pos.right.act = {0.0f, 0.0f, 0.0f};
    foot_pos.left.ref = {-L1+0.001f, 0.001f, -(L2+L4+L5)+0.05f};
    foot_pos.right.ref = {L1-0.001f, 0.001f, -(L2+L4+L5)+0.05f};
    CoM_pos.act = {0.0f, 0.0f, 0.0f};
    CoM_pos.ref = {0.0f, 0.0f, 0.0f};
    ZMP_pos.act = {0.0f, 0.0f, 0.0f};
    ZMP_pos.ref = {0.0f, 0.0f, 0.0f};
}

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