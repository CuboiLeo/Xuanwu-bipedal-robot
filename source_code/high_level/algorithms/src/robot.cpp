#include "robot.h"

void Robot::setMotorData(Motor &motor)
{
    motor.setRefPos(Left_Hip_Yaw, leg_angles.left.ref.hip_yaw);
    motor.setRefPos(Left_Hip_Roll, leg_angles.left.ref.hip_roll);
    motor.setRefPos(Left_Hip_Pitch, leg_angles.left.ref.hip_pitch);
    motor.setRefPos(Left_Knee_Pitch, leg_angles.left.ref.knee_pitch);
    motor.setRefPos(Left_Ankle_Pitch, leg_angles.left.ref.ankle_pitch);
    motor.setRefPos(Right_Hip_Yaw, leg_angles.right.ref.hip_yaw);
    motor.setRefPos(Right_Hip_Roll, leg_angles.right.ref.hip_roll);
    motor.setRefPos(Right_Hip_Pitch, leg_angles.right.ref.hip_pitch);
    motor.setRefPos(Right_Knee_Pitch, leg_angles.right.ref.knee_pitch);
    motor.setRefPos(Right_Ankle_Pitch, leg_angles.right.ref.ankle_pitch);

    motor.setRefVel(Left_Hip_Yaw, leg_velocities.left.ref.hip_yaw);
    motor.setRefVel(Left_Hip_Roll, leg_velocities.left.ref.hip_roll);
    motor.setRefVel(Left_Hip_Pitch, leg_velocities.left.ref.hip_pitch);
    motor.setRefVel(Left_Knee_Pitch, leg_velocities.left.ref.knee_pitch);
    motor.setRefVel(Left_Ankle_Pitch, leg_velocities.left.ref.ankle_pitch);
    motor.setRefVel(Right_Hip_Yaw, leg_velocities.right.ref.hip_yaw);
    motor.setRefVel(Right_Hip_Roll, leg_velocities.right.ref.hip_roll);
    motor.setRefVel(Right_Hip_Pitch, leg_velocities.right.ref.hip_pitch);
    motor.setRefVel(Right_Knee_Pitch, leg_velocities.right.ref.knee_pitch);
    motor.setRefVel(Right_Ankle_Pitch, leg_velocities.right.ref.ankle_pitch);

    motor.setRefTor(Left_Hip_Yaw, leg_torques.left.ref.hip_yaw);
    motor.setRefTor(Left_Hip_Roll, leg_torques.left.ref.hip_roll);
    motor.setRefTor(Left_Hip_Pitch, leg_torques.left.ref.hip_pitch);
    motor.setRefTor(Left_Knee_Pitch, leg_torques.left.ref.knee_pitch);
    motor.setRefTor(Left_Ankle_Pitch, leg_torques.left.ref.ankle_pitch);
    motor.setRefTor(Right_Hip_Yaw, leg_torques.right.ref.hip_yaw);
    motor.setRefTor(Right_Hip_Roll, leg_torques.right.ref.hip_roll);
    motor.setRefTor(Right_Hip_Pitch, leg_torques.right.ref.hip_pitch);
    motor.setRefTor(Right_Knee_Pitch, leg_torques.right.ref.knee_pitch);
    motor.setRefTor(Right_Ankle_Pitch, leg_torques.right.ref.ankle_pitch);
}