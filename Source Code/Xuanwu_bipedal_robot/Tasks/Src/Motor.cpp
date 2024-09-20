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
    const TickType_t TimeIncrement = pdMS_TO_TICKS(1);

		//float count = 0;
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
      uint8_t soft_start_flag = robot.getSoftStartFlag();
			
			if(soft_start_flag != ALL_JOINTS_ZEROED){
				soft_start_flag = dm4310_return_zero_pos();
        robot.setSoftStartFlag(soft_start_flag);
			}
			
			if(soft_start_flag == ALL_JOINTS_ZEROED)
			{
				for(int i = 0; i < joint_num; i++)
				{
//					count += 3.1415926f*0.0005f;
					g_Motor[i].ctrl.kp_set = 5;
					g_Motor[i].ctrl.kd_set = 1;
				}
				
				Joint_Angle ref_left_leg_angles = robot.getRefJointAnglesLeft();
				
				g_Motor[Left_Hip_Yaw].ctrl.pos_set = ref_left_leg_angles.hip_yaw;
				g_Motor[Left_Hip_Roll].ctrl.pos_set = ref_left_leg_angles.hip_roll;
				g_Motor[Left_Hip_Pitch].ctrl.pos_set = ref_left_leg_angles.hip_pitch;
				g_Motor[Left_Knee_Pitch].ctrl.pos_set = ref_left_leg_angles.knee_pitch;
			}

      dm4310_virtual_boundary();
			dm4310_send_all();
			
      vTaskDelayUntil(&xLastWakeTime, TimeIncrement);
    }
  /* USER CODE END Motor_Ctrl_Task */
}
