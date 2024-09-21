#include "Motor.h"
#include "CMSIS_os.h"
#include "Delay.h"
#include "math.h"
#include "Robot_Types.h"

Motor motor;

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
    osDelay(500);
    
    motor.resetJoints();
	osDelay(500);

  /* Infinite loop */
    for(;;)
    {   
		uint8_t soft_start_flag = motor.getSoftStartFlag();    
		if(soft_start_flag != motor.ALL_JOINTS_ZEROED_FLAG){
        	soft_start_flag = motor.returnZeroPos();
            motor.setSoftStartFlag(soft_start_flag);
        }		

      	motor.createVirtualBoundary();
		motor.sendAll();
			
      vTaskDelayUntil(&xLastWakeTime, TimeIncrement);
    }
  /* USER CODE END Motor_Ctrl_Task */
}

Motor::Motor()
{
  	motor_info[Left_Hip_Yaw].can_bus = 1;
	motor_info[Left_Hip_Yaw].id = 0x01;
	motor_info[Left_Hip_Yaw].ctrl.mode = 0; // 0: MIT, 1: Pos, 2: Vel
	motor_info[Left_Hip_Yaw].range = 1.5f;

	motor_info[Left_Hip_Roll].can_bus = 1;
	motor_info[Left_Hip_Roll].id = 0x02;
	motor_info[Left_Hip_Roll].ctrl.mode = 0;
	motor_info[Left_Hip_Roll].range = 0.2f;

	motor_info[Left_Hip_Pitch].can_bus = 1;
	motor_info[Left_Hip_Pitch].id = 0x03;
	motor_info[Left_Hip_Pitch].ctrl.mode = 0;
	motor_info[Left_Hip_Pitch].range = 0.6f;

	motor_info[Left_Knee_Pitch].can_bus = 1;
	motor_info[Left_Knee_Pitch].id = 0x04;
	motor_info[Left_Knee_Pitch].ctrl.mode = 0;
	motor_info[Left_Knee_Pitch].range = 0.6f;

	motor_info[Right_Hip_Yaw].can_bus = 2;
	motor_info[Right_Hip_Yaw].id = 0x05;
	motor_info[Right_Hip_Yaw].ctrl.mode = 0;
	motor_info[Right_Hip_Yaw].range = 1.5f;

	motor_info[Right_Hip_Roll].can_bus = 2;
	motor_info[Right_Hip_Roll].id = 0x06;
	motor_info[Right_Hip_Roll].ctrl.mode = 0;
	motor_info[Right_Hip_Roll].range = 0.2f;

	motor_info[Right_Hip_Pitch].can_bus = 2;
	motor_info[Right_Hip_Pitch].id = 0x07;
	motor_info[Right_Hip_Pitch].ctrl.mode = 0;
	motor_info[Right_Hip_Pitch].range = 0.6f;

	motor_info[Right_Knee_Pitch].can_bus = 2;
	motor_info[Right_Knee_Pitch].id = 0x08;
	motor_info[Right_Knee_Pitch].ctrl.mode = 0;
	motor_info[Right_Knee_Pitch].range = 0.6f;

	soft_start_flag = 0;
}

void Motor::setAllJointsPos(Robot* robot)
{
	for (int i = 0; i < NUM_MOTORS; i++){
		motor_info[i].ctrl.kp_set = 5;
		motor_info[i].ctrl.kd_set = 1;
	}

	Joint_Angle ref_left_leg_angles = robot->getRefJointAnglesLeft();
	motor_info[Left_Hip_Yaw].ctrl.pos_set = ref_left_leg_angles.hip_yaw;
	motor_info[Left_Hip_Roll].ctrl.pos_set = ref_left_leg_angles.hip_roll;
	motor_info[Left_Hip_Pitch].ctrl.pos_set = ref_left_leg_angles.hip_pitch;
	motor_info[Left_Knee_Pitch].ctrl.pos_set = ref_left_leg_angles.knee_pitch;

	Joint_Angle ref_right_leg_angles = robot->getRefJointAnglesRight();
	motor_info[Right_Hip_Yaw].ctrl.pos_set = ref_right_leg_angles.hip_yaw;
	motor_info[Right_Hip_Roll].ctrl.pos_set = ref_right_leg_angles.hip_roll;
	motor_info[Right_Hip_Pitch].ctrl.pos_set = ref_right_leg_angles.hip_pitch;
	motor_info[Right_Knee_Pitch].ctrl.pos_set = ref_right_leg_angles.knee_pitch;
}

uint8_t Motor::returnZeroPos(void)
{
  uint8_t all_joints_reached_pos_flag = 1;
	for(int i = 0; i < NUM_MOTORS; i++)
	{
		motor_info[i].ctrl.pos_set = 0;
		motor_info[i].ctrl.kp_set = 2;
		motor_info[i].ctrl.kd_set = 1;

		if(fabs(motor_info[i].para.pos - motor_info[i].ctrl.pos_set) < 0.2f)
		{
			motor_info[i].reached_pos_flag = 1;
		}
		else
		{
			motor_info[i].reached_pos_flag = 0;
		}

		all_joints_reached_pos_flag = all_joints_reached_pos_flag && motor_info[i].reached_pos_flag;
	}

	if (all_joints_reached_pos_flag)
	{
		for(int i = 0; i < NUM_MOTORS; i++)
		{
			motor_info[i].ctrl.pos_set = 0;
			motor_info[i].ctrl.kp_set = 0;
			motor_info[i].ctrl.kd_set = 0;
		}
	}
	
	return all_joints_reached_pos_flag;
}

void Motor::createVirtualBoundary(void)
{
	for (int i = 0; i < NUM_MOTORS; i++)
	{
		motor_info[i].ctrl.pos_set = (motor_info[i].ctrl.pos_set > motor_info[i].range) ? motor_info[i].range : motor_info[i].ctrl.pos_set;
		motor_info[i].ctrl.pos_set = (motor_info[i].ctrl.pos_set < -motor_info[i].range) ? -motor_info[i].range : motor_info[i].ctrl.pos_set;
			
		if(motor_info[i].para.pos > motor_info[i].range)
		{
			motor_info[i].ctrl.pos_set = motor_info[i].range - 0.1f;
			motor_info[i].ctrl.vel_set = 0;
			motor_info[i].ctrl.tor_set = 0;
			motor_info[i].ctrl.kp_set = 3;
			motor_info[i].ctrl.kd_set = 1;
		}
		else if(motor_info[i].para.pos < -motor_info[i].range)
		{
			motor_info[i].ctrl.pos_set = -motor_info[i].range + 0.1f;
			motor_info[i].ctrl.vel_set = 0;
			motor_info[i].ctrl.tor_set = 0;
			motor_info[i].ctrl.kp_set = 3;
			motor_info[i].ctrl.kd_set = 1;
		}
	}
}

void Motor::resetJoints(void)
{
  	motor.disableAll();
	for (int i = 0; i < NUM_MOTORS; i++)
	{
		dm4310_clear_para(&motor_info[i]);
	}
	motor.enableAll();
}

void Motor::sendAll(void)
{
	dm4310_ctrl_send(&motor_info[Left_Hip_Yaw]);
	dm4310_ctrl_send(&motor_info[Left_Hip_Roll]);
	delay_us(200);
	dm4310_ctrl_send(&motor_info[Left_Hip_Pitch]);
	dm4310_ctrl_send(&motor_info[Left_Knee_Pitch]);
	delay_us(200);
	dm4310_ctrl_send(&motor_info[Right_Hip_Yaw]);
	dm4310_ctrl_send(&motor_info[Right_Hip_Roll]);
	delay_us(200);
	dm4310_ctrl_send(&motor_info[Right_Hip_Pitch]);
	dm4310_ctrl_send(&motor_info[Right_Knee_Pitch]);
	delay_us(200);
}

void Motor::disableAll(void)
{
	dm4310_disable(&motor_info[Left_Hip_Yaw]);
	dm4310_disable(&motor_info[Left_Hip_Roll]);
	delay_us(200);
	dm4310_disable(&motor_info[Left_Hip_Pitch]);
	dm4310_disable(&motor_info[Left_Knee_Pitch]);
	delay_us(200);
	dm4310_disable(&motor_info[Right_Hip_Yaw]);
	dm4310_disable(&motor_info[Right_Hip_Roll]);
	delay_us(200);
	dm4310_disable(&motor_info[Right_Hip_Pitch]);
	dm4310_disable(&motor_info[Right_Knee_Pitch]);
	delay_us(200);
}

void Motor::enableAll(void)
{
	dm4310_enable(&motor_info[Left_Hip_Yaw]);
	dm4310_enable(&motor_info[Left_Hip_Roll]);
	delay_us(200);
	dm4310_enable(&motor_info[Left_Hip_Pitch]);
	dm4310_enable(&motor_info[Left_Knee_Pitch]);
	delay_us(200);
	dm4310_enable(&motor_info[Right_Hip_Yaw]);
	dm4310_enable(&motor_info[Right_Hip_Roll]);
	delay_us(200);
	dm4310_enable(&motor_info[Right_Hip_Pitch]);
	dm4310_enable(&motor_info[Right_Knee_Pitch]);
}

void fdcan1_rx_callback(void)
{
	uint16_t rec_id;
	uint8_t rx_data[8] = {0};
	fdcanx_receive(&hfdcan1, &rec_id, rx_data);
	switch (rec_id)
	{
 		case 0x11: dm4310_fbdata(&motor.motor_info[Left_Hip_Yaw], rx_data); break;
		case 0x12: dm4310_fbdata(&motor.motor_info[Left_Hip_Roll], rx_data); break;
		case 0x13: dm4310_fbdata(&motor.motor_info[Left_Hip_Pitch], rx_data); break;
		case 0x14: dm4310_fbdata(&motor.motor_info[Left_Knee_Pitch], rx_data); break;
	}
}

void fdcan2_rx_callback(void)
{
	uint16_t rec_id;
	uint8_t rx_data[8] = {0};
	fdcanx_receive(&hfdcan2, &rec_id, rx_data);
	switch (rec_id)
	{
		case 0x15: dm4310_fbdata(&motor.motor_info[Right_Hip_Yaw], rx_data); break;
		case 0x16: dm4310_fbdata(&motor.motor_info[Right_Hip_Roll], rx_data); break;
		case 0x17: dm4310_fbdata(&motor.motor_info[Right_Hip_Pitch], rx_data); break;
		case 0x18: dm4310_fbdata(&motor.motor_info[Right_Knee_Pitch], rx_data); break;
	}
}
