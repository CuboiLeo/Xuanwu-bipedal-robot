#include "ET16S_Remote.h"
#include "usart.h"
#include "cmsis_os.h"
#include "Robot.h"
#include "Forward_Kinematics.h"
#include "DM4310ctrl.h"

Robot_t g_Robot;

void Robot_Task(void const * argument)
{
    portTickType xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();
    const TickType_t TimeIncrement = pdMS_TO_TICKS(2);

    Remote_Init(&REMOTE_UART);
    for(;;)
    {
        Robot_Joint_Angle_Assign();
		Forward_Kinematics_DH();
        vTaskDelayUntil(&xLastWakeTime, TimeIncrement);
    }
}

void Robot_Joint_Angle_Assign(void)
{
    g_Robot.left_leg.hip_yaw = g_Motor[Left_Hip_Yaw].para.pos;
    g_Robot.left_leg.hip_roll = g_Motor[Left_Hip_Roll].para.pos;
    g_Robot.left_leg.hip_pitch = g_Motor[Left_Hip_Pitch].para.pos;
    g_Robot.left_leg.knee_pitch = g_Motor[Left_Knee_Pitch].para.pos;
    g_Robot.right_leg.hip_yaw = g_Motor[Right_Hip_Yaw].para.pos;
    g_Robot.right_leg.hip_roll = g_Motor[Right_Hip_Roll].para.pos;
    g_Robot.right_leg.hip_pitch = g_Motor[Right_Hip_Pitch].para.pos;
    g_Robot.right_leg.knee_pitch = g_Motor[Right_Knee_Pitch].para.pos;
}
