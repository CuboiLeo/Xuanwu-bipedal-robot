#include "Motor.h"
#include "CMSIS_os.h"
#include "delay.h"

void Motor_Ctrl_Task(void const * argument)
{
  /* USER CODE BEGIN Motor_Ctrl_Task */
    portTickType xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();
    const TickType_t TimeIncrement = pdMS_TO_TICKS(2000);

    can_bsp_init();
	delay_init(480);
	power(1);
  osDelay(500);
	dm4310_motor_init();
	osDelay(500);
	
	dm4310_enable(&hfdcan1, &motor[Motor1]);
	delay_us(200);
	dm4310_enable(&hfdcan1, &motor[Motor2]);
	delay_us(200);


  /* Infinite loop */
    for(;;)
    {

			dm4310_ctrl_send(&hfdcan1, &motor[Motor1]);
		dm4310_ctrl_send(&hfdcan1, &motor[Motor2]);
		delay_us(200);
//		dm4310_ctrl_send(&hfdcan1, &motor[Motor3]);
//		dm4310_ctrl_send(&hfdcan1, &motor[Motor4]);
//			delay_us(200);
//			dm4310_ctrl_send(&hfdcan2, &motor[Motor5]);
//		dm4310_ctrl_send(&hfdcan2, &motor[Motor6]);
//		delay_us(200);
//		dm4310_ctrl_send(&hfdcan2, &motor[Motor7]);
//		dm4310_ctrl_send(&hfdcan2, &motor[Motor8]);
			
        vTaskDelayUntil(&xLastWakeTime, TimeIncrement);
    }
  /* USER CODE END Motor_Ctrl_Task */
}