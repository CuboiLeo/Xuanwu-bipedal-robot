#include "Task_Manager.h"
#include "Robot.h"
#include "Motor.h"
#include "ET16S_Remote.h"
#include "IMU.h"
#include "Onboard_Buzzer.h"
#include "debug.h"
#include <algorithm>
#include "Orin_NX.h"

Robot robot;
Motor motor;
Remote remote;
IMU imu;
Buzzer buzzer;
Orin orin;
uint32_t last_time;
uint32_t curr_time;
uint32_t elapsed_time;

void Robot_Task(void *argument)
{
    portTickType xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();
    const TickType_t TimeIncrement = pdMS_TO_TICKS(1);

    remote.Init(&REMOTE_UART); // Initialize the remote

    for (;;)
    {
        remote.checkWatchdog(&REMOTE_UART);

        robot.setRefRobotVel({remote.getLeftStickX() / Remote::CHANNEL_MAX_VALUE * ROBOT_MAX_VEL, remote.getLeftStickY() / Remote::CHANNEL_MAX_VALUE * ROBOT_MAX_VEL, 0});
        robot.setRefRobotAngVel({0, 0, remote.getRightStickX() / Remote::CHANNEL_MAX_VALUE * ROBOT_MAX_ANG_VEL});

        if (motor.getSoftStartFlag() == motor.ALL_JOINTS_ZEROED_FLAG)
        {
#ifdef USE_LITE_PACKAGE
            orin.decodeDataLite(motor);
#else
            orin.decodeData(motor);
#endif
        }

        vTaskDelayUntil(&xLastWakeTime, TimeIncrement);
    }
}

void Motor_Ctrl_Task(void *argument)
{
    /* USER CODE BEGIN Motor_Ctrl_Task */
    portTickType xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();
    const TickType_t TimeIncrement = pdMS_TO_TICKS(1);

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
        else
            motor.createVirtualBoundary();

        motor.sendAll();
				curr_time = HAL_GetTick();
				elapsed_time = curr_time - last_time;
				last_time = curr_time;

        vTaskDelayUntil(&xLastWakeTime, TimeIncrement);
    }
    /* USER CODE END Motor_Ctrl_Task */
}

void Debug_Task(void *argument)
{
    portTickType xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();
    const TickType_t TimeIncrement = pdMS_TO_TICKS(10);

    HAL_UART_Init(&huart7);

    for (;;)
    {
        char buffer[128]; // Adjust the size based on the data you need to print

        // Format the string using sprintf
        sprintf(buffer, "/*%d, %f, %f*/\n", elapsed_time, motor.getRefPos(Left_Knee_Pitch), motor.getPos(Left_Knee_Pitch));

        // Transmit the formatted string over UART
        HAL_UART_Transmit(&huart7, (uint8_t *)buffer, strlen(buffer), 0xFFFF);

        vTaskDelayUntil(&xLastWakeTime, TimeIncrement);
    }
}

// External semaphore handle (from FreeRTOS)
extern osSemaphoreId imuBinarySem01Handle;
void IMU_Task(void *argument)
{
    portTickType xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();
    const TickType_t TimeIncrement = pdMS_TO_TICKS(1);

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
    const TickType_t TimeIncrement = pdMS_TO_TICKS(5000);

    robot.initBatteryADC();
    buzzer.Init();

    for (;;)
    {
        float battery_voltage = robot.getBatteryVoltage();
        //  Low battery voltage warning
        if (battery_voltage < 22.2f)
        {
            buzzer.Beep();
        }

        vTaskDelayUntil(&xLastWakeTime, TimeIncrement);
    }
}

void Orin_Task(void *argument)
{
    portTickType xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();
    const TickType_t TimeIncrement = pdMS_TO_TICKS(1);

    for (;;)
    {
#ifdef USE_LITE_PACKAGE
        orin.sendDataLite(motor, imu, robot.getRefRobotVel(), robot.getRefRobotAngVel());
#else
        orin.sendData(motor, imu, robot.getRefRobotVel(), robot.getRefRobotAngVel());
#endif
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
    case 0x15:
        dm4310_fbdata(&motor.motor_info[Left_Ankle_Pitch], rx_data);
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
    case 0x31:
        dm4310_fbdata(&motor.motor_info[Right_Hip_Yaw], rx_data);
        break;
    case 0x32:
        dm4310_fbdata(&motor.motor_info[Right_Hip_Roll], rx_data);
        break;
    case 0x33:
        dm4310_fbdata(&motor.motor_info[Right_Hip_Pitch], rx_data);
        break;
    case 0x34:
        dm4310_fbdata(&motor.motor_info[Right_Knee_Pitch], rx_data);
        break;
    case 0x35:
        dm4310_fbdata(&motor.motor_info[Right_Ankle_Pitch], rx_data);
        break;
    }
}

void fdcan3_rx_callback(void)
{
#ifdef USE_LITE_PACKAGE
    orin.receiveDataLite();
#else
    orin.receiveData();
#endif
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
