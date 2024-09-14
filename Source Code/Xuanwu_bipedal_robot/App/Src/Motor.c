#include "Motor.h"
#include "CMSIS_os.h"
#include "Delay.h"
#include "Robot.h"

void Motor_Ctrl_Task(void const * argument)
{
  /* USER CODE BEGIN Motor_Ctrl_Task */
    portTickType xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();
    const TickType_t TimeIncrement = pdMS_TO_TICKS(2);

    can_bsp_init();
    delay_init(480);
    osDelay(100);
    FDCAN1_PowerUp(GPIO_PIN_SET);
    FDCAN2_PowerUp(GPIO_PIN_SET);
    osDelay(500);
    dm4310_motor_init();
    osDelay(500);
    
    dm4310_reset_joints();
		osDelay(500);
  /* Infinite loop */
    for(;;)
    {
			if(g_Robot.soft_start_flag != ALL_JOINTS_ZEROED){
				g_Robot.soft_start_flag = dm4310_return_zero_pos();
			}
			
			if(g_Robot.soft_start_flag == ALL_JOINTS_ZEROED)
			{
				
			}

      dm4310_virtual_boundary();
			dm4310_send_all();
			
      vTaskDelayUntil(&xLastWakeTime, TimeIncrement);
    }
  /* USER CODE END Motor_Ctrl_Task */
}
