#include "main.h"
#include "cmsis_os.h"
#include "BMI088driver.h"
#include "gpio.h"
#include "tim.h"
#include "Fusion.h"
#include "IMU.h"

#define DES_TEMP    40.0f
#define MAX_OUT     500

float gyro[3], accel[3], temp;
extern osSemaphoreId imuBinarySem01Handle;
FusionAhrs IMU_AHRS;
IMU_t g_IMU;

void IMU_Task(void const * argument)
{
		portTickType xLastWakeTime;
		xLastWakeTime = xTaskGetTickCount();
		const TickType_t TimeIncrement = pdMS_TO_TICKS(10);

    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
		FusionAhrsInitialise(&IMU_AHRS);
    while(BMI088_init())
    {
        ;
    }
    for(;;)
    {
        osSemaphoreWait(imuBinarySem01Handle, osWaitForever);
        
        BMI088_read(gyro, accel, &temp);
			
				if(DES_TEMP > temp)
					htim3.Instance->CCR4 = MAX_OUT;
				else
					htim3.Instance->CCR4 = 0;
				
			  const FusionVector Accel = {accel[0]/9.81f,accel[1]/9.81f,accel[2]/9.81f};
				const FusionVector Gyro = {gyro[0]*57.2958f,gyro[1]*57.2958f,gyro[2]*57.2958f};
				FusionAhrsUpdateNoMagnetometer(&IMU_AHRS, Gyro, Accel, 0.001f);
				const FusionEuler IMU_Euler = FusionQuaternionToEuler(FusionAhrsGetQuaternion(&IMU_AHRS));
				
        g_IMU.ax = accel[0];
        g_IMU.ay = accel[1];
        g_IMU.az = accel[2];
        g_IMU.gx = gyro[0];
        g_IMU.gy = gyro[1];
        g_IMU.gz = gyro[2];
				g_IMU.yaw_deg = IMU_Euler.angle.yaw;
				g_IMU.pitch_deg = IMU_Euler.angle.pitch;
				g_IMU.roll_deg = IMU_Euler.angle.roll;
        g_IMU.yaw_rad = IMU_Euler.angle.yaw * 0.0174533f;
        g_IMU.pitch_rad = IMU_Euler.angle.pitch * 0.0174533f;
        g_IMU.roll_rad = IMU_Euler.angle.roll * 0.0174533f;
        g_IMU.temp = temp;
				
				vTaskDelayUntil(&xLastWakeTime, TimeIncrement);
    }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if(GPIO_Pin == ACC_INT_Pin)
    {
        osSemaphoreRelease(imuBinarySem01Handle);
    }
    else if(GPIO_Pin == GYRO_INT_Pin)
    {

    }
}
