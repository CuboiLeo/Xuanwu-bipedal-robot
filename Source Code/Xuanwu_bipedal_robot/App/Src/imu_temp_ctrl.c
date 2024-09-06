#include "main.h"
#include "cmsis_os.h"
#include "BMI088driver.h"
#include "gpio.h"
#include "tim.h"
#include "Fusion.h"
#include "IMU.h"

#define DES_TEMP    40.0f
#define MAX_OUT     10000

float gyro[3], accel[3], temp, curr_time, prev_time, sample_period, quat[4] = {1.0f,0.0f,0.0f,0.0f};
extern osSemaphoreId imuBinarySem01Handle;
FusionAhrs IMU_AHRS;
extern IMU_t g_IMU;

/**
************************************************************************
* @brief:      	IMU_TempCtrlTask(void const * argument)
* @param:       argument - 任务参数
* @retval:     	void
* @details:    	IMU温度控制任务函数
************************************************************************
**/
void IMU_Task(void const * argument)
{
    osDelay(500);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
		FusionAhrsInitialise(&IMU_AHRS);
    while(BMI088_init())
    {
        ;
    }
    for (;;)
    {
        osSemaphoreWait(imuBinarySem01Handle, osWaitForever);
        
        BMI088_read(gyro, accel, &temp);
			
				if(DES_TEMP > temp)
					htim3.Instance->CCR4 = MAX_OUT;
				else
					htim3.Instance->CCR4 = 0;
				
				curr_time = HAL_GetTick() / 1000.0f;
				sample_period = curr_time - prev_time;
				prev_time = curr_time;
			  const FusionVector Accel = {accel[0]/9.81f,accel[1]/9.81f,accel[2]/9.81f};
				const FusionVector Gyro = {gyro[0]*57.2958f,gyro[1]*57.2958f,gyro[2]*57.2958f};
				FusionAhrsUpdateNoMagnetometer(&IMU_AHRS, Gyro, Accel, sample_period);
				const FusionEuler IMU_Euler = FusionQuaternionToEuler(FusionAhrsGetQuaternion(&IMU_AHRS));
				
				g_IMU.yaw = IMU_Euler.angle.yaw;
				g_IMU.pitch = IMU_Euler.angle.pitch;
				g_IMU.roll = IMU_Euler.angle.roll;
    }
}
/**
************************************************************************
* @brief:      	HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
* @param:       GPIO_Pin - 触发中断的GPIO引脚
* @retval:     	void
* @details:    	GPIO外部中断回调函数，处理加速度计和陀螺仪中断
************************************************************************
**/
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
