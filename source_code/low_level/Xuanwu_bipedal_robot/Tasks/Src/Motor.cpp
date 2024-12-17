#include "Motor.h"
#include "math.h"
#include "Robot_Types.h"

Motor::Motor()
{
  motor_info[Left_Hip_Yaw].can_bus = 1;
	motor_info[Left_Hip_Yaw].id = 0x01;
	motor_info[Left_Hip_Yaw].ctrl.mode = 0; // 0: MIT, 1: Pos, 2: Vel
	motor_info[Left_Hip_Yaw].range = 0.3f;

	motor_info[Left_Hip_Roll].can_bus = 1;
	motor_info[Left_Hip_Roll].id = 0x02;
	motor_info[Left_Hip_Roll].ctrl.mode = 0;
	motor_info[Left_Hip_Roll].range = 0.4f;

	motor_info[Left_Hip_Pitch].can_bus = 1;
	motor_info[Left_Hip_Pitch].id = 0x03;
	motor_info[Left_Hip_Pitch].ctrl.mode = 0;
	motor_info[Left_Hip_Pitch].range = 1.5f;

	motor_info[Left_Knee_Pitch].can_bus = 1;
	motor_info[Left_Knee_Pitch].id = 0x04;
	motor_info[Left_Knee_Pitch].ctrl.mode = 0;
	motor_info[Left_Knee_Pitch].range = 1.5f;

	motor_info[Right_Hip_Yaw].can_bus = 2;
	motor_info[Right_Hip_Yaw].id = 0x05;
	motor_info[Right_Hip_Yaw].ctrl.mode = 0;
	motor_info[Right_Hip_Yaw].range = 0.3f;

	motor_info[Right_Hip_Roll].can_bus = 2;
	motor_info[Right_Hip_Roll].id = 0x06;
	motor_info[Right_Hip_Roll].ctrl.mode = 0;
	motor_info[Right_Hip_Roll].range = 0.4f;

	motor_info[Right_Hip_Pitch].can_bus = 2;
	motor_info[Right_Hip_Pitch].id = 0x07;
	motor_info[Right_Hip_Pitch].ctrl.mode = 0;
	motor_info[Right_Hip_Pitch].range = 1.5f;

	motor_info[Right_Knee_Pitch].can_bus = 2;
	motor_info[Right_Knee_Pitch].id = 0x08;
	motor_info[Right_Knee_Pitch].ctrl.mode = 0;
	motor_info[Right_Knee_Pitch].range = 1.5f;

	soft_start_flag = 0;
}

uint8_t Motor::returnZeroPos(void)
{
  uint8_t all_joints_reached_pos_flag = 1;
	for(int i = 0; i < NUM_MOTORS; i++)
	{
		motor_info[i].ctrl.pos_set = 0;
		motor_info[i].ctrl.kp_set = 5;
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
			motor_info[i].ctrl.kp_set = 50;
			motor_info[i].ctrl.kd_set = 2;
			motor_info[i].ctrl.tor_set = motor_info[i].ctrl.pos_set;
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
			motor_info[i].ctrl.kp_set = 10;
			motor_info[i].ctrl.kd_set = 2;
		}
		else if(motor_info[i].para.pos < -motor_info[i].range)
		{
			motor_info[i].ctrl.pos_set = -motor_info[i].range + 0.1f;
			motor_info[i].ctrl.vel_set = 0;
			motor_info[i].ctrl.tor_set = 0;
			motor_info[i].ctrl.kp_set = 10;
			motor_info[i].ctrl.kd_set = 2;
		}
	}
}

void Motor::resetJoints(void)
{
  	disableAll();
	for (int i = 0; i < NUM_MOTORS; i++)
	{
		dm4310_clear_para(&motor_info[i]);
	}
	enableAll();
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
