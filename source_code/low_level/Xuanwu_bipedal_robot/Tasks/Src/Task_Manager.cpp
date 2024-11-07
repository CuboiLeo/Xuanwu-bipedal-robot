#include "Task_Manager.h"
#include "Robot.h"
#include "Kinematics.h"
#include "Motor.h"
#include "ET16S_Remote.h"
#include "IMU.h"
#include "Onboard_Buzzer.h"
#include "debug.h"
#include <algorithm>
#include "Dynamics.h"
#include "Controls.h"
#include "Orin_NX.h"

Robot robot;
Kinematics kinematics;
Motor motor;
Remote remote;
IMU imu;
Buzzer buzzer;
Dynamics dynamics;
Controls controls;
Orin orin;

void Robot_Task(void *argument)
{
    portTickType xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();
    const TickType_t TimeIncrement = pdMS_TO_TICKS(2);

    remote.Init(&REMOTE_UART); // Initialize the remote

    for (;;)
    {
        remote.checkWatchdog(&REMOTE_UART);
        // Assign joint angles based on motor feedback
        Joint_Angle left_angles = {
            motor.getPos(Left_Hip_Yaw),
            motor.getPos(Left_Hip_Roll),
            motor.getPos(Left_Hip_Pitch),
            motor.getPos(Left_Knee_Pitch)};
        Joint_Angle right_angles = {
            motor.getPos(Right_Hip_Yaw),
            motor.getPos(Right_Hip_Roll),
            motor.getPos(Right_Hip_Pitch),
            motor.getPos(Right_Knee_Pitch)};

        // Update the robot's joint angles
        robot.setActJointAnglesLeft(left_angles);
        robot.setActJointAnglesRight(right_angles);

        left_angles.hip_roll += LEFT_LEG_HIP_ROLL_OFFSET;
        right_angles.hip_roll += RIGHT_LEG_HIP_ROLL_OFFSET;
        robot.setActCoMPos(dynamics.computeCenterOfMass(left_angles, right_angles, imu.getRotationMatrix()));
        robot.setActZMPPos(dynamics.computeZeroMomentPoint(imu.getAccel(), imu.getGyro(), imu.getGyroDot(), imu.getRotationMatrix()));

        // Compute the robot's foot positions using forward kinematics
        robot.setActFootPosLeft(kinematics.computeForwardKinematics(left_angles, LEFT_LEG));
        robot.setActFootPosRight(kinematics.computeForwardKinematics(right_angles, RIGHT_LEG));

        // Adjust the robot's CoM position
        // Direction_Vector_Two ref_foot_pos = controls.controlCoMPos(robot.getRefCoMPos(), robot.getActCoMPos(), robot.getRefFootPosLeft(), robot.getRefFootPosRight());
        // robot.setRefFootPosLeft(ref_foot_pos.left);
        // robot.setRefFootPosRight(ref_foot_pos.right);

        // Update the robot's velocity
        Direction_Vector ref_robot_vel = {remote.getLeftStickX() / remote.CHANNEL_MAX_VALUE * 0.1f, remote.getLeftStickY() / remote.CHANNEL_MAX_VALUE * 0.1f, 0.0f};
        robot.setRefRobotVel(ref_robot_vel);

        //Update the robot's reference foot positions
        robot.setRefFootPosLeft(kinematics.generateTrajectory(ref_robot_vel, LEFT_LEG));
        robot.setRefFootPosRight(kinematics.generateTrajectory(ref_robot_vel, RIGHT_LEG));
				
        // Compute the robot's joint angles using inverse kinematics
        robot.setRefJointAnglesLeft(kinematics.computeInverseKinematics(robot.getRefFootPosLeft(), robot.getActFootPosLeft(), robot.getActJointAnglesLeft(), LEFT_LEG));
        robot.setRefJointAnglesRight(kinematics.computeInverseKinematics(robot.getRefFootPosRight(), robot.getActFootPosRight(), robot.getActJointAnglesRight(), RIGHT_LEG));

        if (motor.getSoftStartFlag() == motor.ALL_JOINTS_ZEROED_FLAG)
        {
            motor.setAllJointsPos(&robot);
        }

        vTaskDelayUntil(&xLastWakeTime, TimeIncrement);
    }
}

void Motor_Ctrl_Task(void *argument)
{
    /* USER CODE BEGIN Motor_Ctrl_Task */
    portTickType xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();
    const TickType_t TimeIncrement = pdMS_TO_TICKS(2);

    // float count = 0;
    can_bsp_init();
    delay_init(480);
    osDelay(100);
    FDCAN1_PowerUp(GPIO_PIN_SET);
    FDCAN2_PowerUp(GPIO_PIN_SET);
    osDelay(500);
    osDelay(500);

    motor.resetJoints();
    osDelay(500);

    /* Infinite loop */
    for (;;)
    {
        uint8_t soft_start_flag = motor.getSoftStartFlag();
        if (soft_start_flag != motor.ALL_JOINTS_ZEROED_FLAG)
        {
            soft_start_flag = motor.returnZeroPos();
            motor.setSoftStartFlag(soft_start_flag);
        }

        motor.createVirtualBoundary();
        motor.sendAll();

        vTaskDelayUntil(&xLastWakeTime, TimeIncrement);
    }
    /* USER CODE END Motor_Ctrl_Task */
}

void Debug_Task(void *argument)
{
    portTickType xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();
    const TickType_t TimeIncrement = pdMS_TO_TICKS(100);

    HAL_UART_Init(&huart7);

    for (;;)
    {
//        char buffer[128]; // Adjust the size based on the data you need to print

//        // Format the string using sprintf
//        sprintf(buffer, "/*%f, %f, %f, %f*/\n",
//                robot.getRefFootPosLeft().y, robot.getActFootPosLeft().y,robot.getRefFootPosLeft().z, robot.getActFootPosLeft().z);

//        // Transmit the formatted string over UART
//        HAL_UART_Transmit(&huart7, (uint8_t *)buffer, strlen(buffer), 0xFFFF);

        vTaskDelayUntil(&xLastWakeTime, TimeIncrement);
    }
}

// External semaphore handle (from FreeRTOS)
extern osSemaphoreId imuBinarySem01Handle;
void IMU_Task(void *argument)
{
    portTickType xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();
    const TickType_t TimeIncrement = pdMS_TO_TICKS(2);

    float gyro[3], accel[3], temperature;
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
    while (BMI088_init())
    {
        // Waiting for sensor to initialize
    }
    for (;;)
    {
        osSemaphoreAcquire(imuBinarySem01Handle, osWaitForever);

        // Read IMU data
        BMI088_read(gyro, accel, &temperature);
        // Keep the IMU at a constant temperature
        imu.heatControl();
        // Update IMU object with sensor values
        imu.updateRaw(accel, gyro, temperature);
        // Process IMU to update orientation
        imu.processData();

        vTaskDelayUntil(&xLastWakeTime, TimeIncrement);
    }
}

void System_Monitor_Task(void *argument)
{
    portTickType xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();
    const TickType_t TimeIncrement = pdMS_TO_TICKS(1);

    robot.initBatteryADC();
    //buzzer.Init();

    for (;;)
    {
        //float battery_voltage = robot.getBatteryVoltage();
        // Low battery voltage warning
        //if (battery_voltage < 22.2f)
        //{
            //buzzer.Beep();
        //}
        orin.sendData();

        vTaskDelayUntil(&xLastWakeTime, TimeIncrement);
    }
}

void fdcan1_rx_callback(void)
{
    uint16_t rec_id;
    uint8_t rx_data[8] = {0};
    fdcanx_receive(&hfdcan1, &rec_id, rx_data);
    switch (rec_id)
    {
    case 0x11:
        dm4310_fbdata(&motor.motor_info[Left_Hip_Yaw], rx_data);
        break;
    case 0x12:
        dm4310_fbdata(&motor.motor_info[Left_Hip_Roll], rx_data);
        break;
    case 0x13:
        dm4310_fbdata(&motor.motor_info[Left_Hip_Pitch], rx_data);
        break;
    case 0x14:
        dm4310_fbdata(&motor.motor_info[Left_Knee_Pitch], rx_data);
        break;
    }
}

void fdcan2_rx_callback(void)
{
    uint16_t rec_id;
    uint8_t rx_data[8] = {0};
    fdcanx_receive(&hfdcan2, &rec_id, rx_data);
    switch (rec_id)
    {
    case 0x15:
        dm4310_fbdata(&motor.motor_info[Right_Hip_Yaw], rx_data);
        break;
    case 0x16:
        dm4310_fbdata(&motor.motor_info[Right_Hip_Roll], rx_data);
        break;
    case 0x17:
        dm4310_fbdata(&motor.motor_info[Right_Hip_Pitch], rx_data);
        break;
    case 0x18:
        dm4310_fbdata(&motor.motor_info[Right_Knee_Pitch], rx_data);
        break;
    }
}

void fdcan3_rx_callback(void)
{
    orin.receiveData();
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == ACC_INT_Pin)
    {
        osSemaphoreRelease(imuBinarySem01Handle);
    }
    else if (GPIO_Pin == GYRO_INT_Pin)
    {
    }
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart->Instance == REMOTE_UART.Instance)
    {
        remote.processBuffer();
        // enable uart receive for next data frame
        HAL_UARTEx_ReceiveToIdle_DMA(huart, remote.remote_buffer, remote.REMOTE_BUFFER_SIZE);
        // still disable half transfer interrupt (@ref void UART_Service_Init(void))
        __HAL_DMA_DISABLE_IT(REMOTE_UART.hdmarx, DMA_IT_HT);
    }
}
