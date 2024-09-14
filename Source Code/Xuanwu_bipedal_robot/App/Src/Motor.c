#include "Motor.h"
#include "CMSIS_os.h"
#include "Delay.h"
#include "Robot.h"
#include "math.h"

void Motor_Ctrl_Task(void const * argument)
{
  /* USER CODE BEGIN Motor_Ctrl_Task */
    portTickType xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();
    const TickType_t TimeIncrement = pdMS_TO_TICKS(2);

		float count = 0;
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
				for(int i = 0; i < joint_num; i++)
				{
					count += 3.1415926f*0.0005f;
					g_Motor[i].ctrl.pos_set = 0.4f*sinf(count);
					g_Motor[i].ctrl.kp_set = 5;
					g_Motor[i].ctrl.kd_set = 1;
				}
			}

      dm4310_virtual_boundary();
			dm4310_send_all();
			
      vTaskDelayUntil(&xLastWakeTime, TimeIncrement);
    }
  /* USER CODE END Motor_Ctrl_Task */
}
