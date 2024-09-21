#include "usart.h"
#include "cmsis_os.h"
#include "Robot.h"
#include "ET16S_Remote.h"
#include "Kinematics.h"
#include "Motor.h"

Robot robot;
// FreeRTOS Task for controlling the robot
void Robot_Task(void const* argument) {
    portTickType xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();
    const TickType_t TimeIncrement = pdMS_TO_TICKS(1);

    robot.getRemote()->Init(&REMOTE_UART);  // Initialize the remote

    for (;;) {
        // Assign joint angles based on motor feedback
        Joint_Angle left_angles = {
            robot.getMotor()->getPos(Left_Hip_Yaw),
            robot.getMotor()->getPos(Left_Hip_Roll),
            robot.getMotor()->getPos(Left_Hip_Pitch),
            robot.getMotor()->getPos(Left_Knee_Pitch)
        };
        Joint_Angle right_angles = {
            robot.getMotor()->getPos(Right_Hip_Yaw),
            robot.getMotor()->getPos(Right_Hip_Roll),
            robot.getMotor()->getPos(Right_Hip_Pitch),
            robot.getMotor()->getPos(Right_Knee_Pitch)
        };

        // Update the robot's joint angles
        robot.setActJointAnglesLeft(left_angles);
        robot.setActJointAnglesRight(right_angles);

        robot.getKinematics()->computeForwardKinematics(&robot);
        robot.getKinematics()->computeInverseKinematics(&robot);

        if(robot.getMotor()->getSoftStartFlag() == robot.getMotor()->ALL_JOINTS_ZEROED_FLAG) {
            robot.getMotor()->setAllJointsPos(&robot);
        } 

        vTaskDelayUntil(&xLastWakeTime, TimeIncrement);
    }
}

Robot::Robot(): kinematics(), motor(), remote() {
    act_left_leg_angles = {0.0f, 0.0f, 0.0f, 0.0f};
    act_right_leg_angles = {0.0f, 0.0f, 0.0f, 0.0f};
    ref_left_leg_angles = {0.0f, 0.0f, 0.0f, 0.0f};
    ref_right_leg_angles = {0.0f, 0.0f, 0.0f, 0.0f};
    ref_left_foot_pos = {-0.095f, 0.0f, -0.559999f };
    ref_right_foot_pos = {0.0f, 0.0f, 0.0f};
    act_left_foot_pos = {0.0f, 0.0f, 0.0f};
    act_right_foot_pos = {0.0f, 0.0f, 0.0f};
}
