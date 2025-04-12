#include "Motor.h"
#include "math.h"
#include "Robot_Types.h"

Motor::Motor()
{
	motor_info[Left_Hip_Yaw].can_bus = 1;
	motor_info[Left_Hip_Yaw].id = 0x01;
	motor_info[Left_Hip_Yaw].ctrl.mode = 0; // 0: MIT, 1: Pos, 2: Vel
	motor_info[Left_Hip_Yaw].range = 0.5f;

	motor_info[Left_Hip_Roll].can_bus = 1;
	motor_info[Left_Hip_Roll].id = 0x02;
	motor_info[Left_Hip_Roll].ctrl.mode = 0;
	motor_info[Left_Hip_Roll].range = 0.7f;

	motor_info[Left_Hip_Pitch].can_bus = 1;
	motor_info[Left_Hip_Pitch].id = 0x03;
	motor_info[Left_Hip_Pitch].ctrl.mode = 0;
	motor_info[Left_Hip_Pitch].range = 1.5f;

	motor_info[Left_Knee_Pitch].can_bus = 1;
	motor_info[Left_Knee_Pitch].id = 0x04;
	motor_info[Left_Knee_Pitch].ctrl.mode = 0;
	motor_info[Left_Knee_Pitch].range = 1.5f;

	motor_info[Left_Ankle_Pitch].can_bus = 1;
	motor_info[Left_Ankle_Pitch].id = 0x05;
	motor_info[Left_Ankle_Pitch].ctrl.mode = 0;
	motor_info[Left_Ankle_Pitch].range = 1.5f;

	motor_info[Right_Hip_Yaw].can_bus = 2;
	motor_info[Right_Hip_Yaw].id = 0x21;
	motor_info[Right_Hip_Yaw].ctrl.mode = 0;
	motor_info[Right_Hip_Yaw].range = 0.5f;

	motor_info[Right_Hip_Roll].can_bus = 2;
	motor_info[Right_Hip_Roll].id = 0x22;
	motor_info[Right_Hip_Roll].ctrl.mode = 0;
	motor_info[Right_Hip_Roll].range = 0.7f;

	motor_info[Right_Hip_Pitch].can_bus = 2;
	motor_info[Right_Hip_Pitch].id = 0x23;
	motor_info[Right_Hip_Pitch].ctrl.mode = 0;
	motor_info[Right_Hip_Pitch].range = 1.5f;

	motor_info[Right_Knee_Pitch].can_bus = 2;
	motor_info[Right_Knee_Pitch].id = 0x24;
	motor_info[Right_Knee_Pitch].ctrl.mode = 0;
	motor_info[Right_Knee_Pitch].range = 1.5f;

	motor_info[Right_Ankle_Pitch].can_bus = 2;
	motor_info[Right_Ankle_Pitch].id = 0x25;
	motor_info[Right_Ankle_Pitch].ctrl.mode = 0;
	motor_info[Right_Ankle_Pitch].range = 1.5f;

	soft_start_flag = 0;
}

uint8_t Motor::returnZeroPos(void)
{
	all_joints_zeroed_flag = 1;
	for (int i = 0; i < NUM_MOTORS; i++)
	{
		motor_info[i].ctrl.pos_set = 0;
		motor_info[Left_Hip_Pitch].ctrl.pos_set = 0.152f;
		motor_info[Left_Knee_Pitch].ctrl.pos_set = -0.36f;
		motor_info[Left_Ankle_Pitch].ctrl.pos_set = 0.208f;
		motor_info[Right_Hip_Pitch].ctrl.pos_set = 0.152f;
		motor_info[Right_Knee_Pitch].ctrl.pos_set = -0.36f;
		motor_info[Right_Ankle_Pitch].ctrl.pos_set = 0.208f;
		
		motor_info[i].ctrl.kp_set = 10.0f;
		motor_info[i].ctrl.kd_set = 1.0f;
		
		if (fabs(motor_info[i].para.pos - motor_info[i].ctrl.pos_set) < 0.2f)
		{
			motor_info[i].reached_pos_flag = 1;
		}
		else
		{
			motor_info[i].reached_pos_flag = 0;
		}

		all_joints_zeroed_flag = all_joints_zeroed_flag && motor_info[i].reached_pos_flag;
	}

	if (all_joints_zeroed_flag)
	{
		for (int i = 0; i < NUM_MOTORS; i++)
		{
			// motor_info[i].ctrl.kp_set = kps[i];
			// motor_info[i].ctrl.kd_set = kds[i];
			motor_info[i].ctrl.kp_set = 0.0f;
			motor_info[i].ctrl.kd_set = 0.0f;
		}
	}

	return all_joints_zeroed_flag;
}

void Motor::createVirtualBoundary(void)
{
	for (int i = 0; i < NUM_MOTORS; i++)
	{
		motor_info[i].ctrl.pos_set = (motor_info[i].ctrl.pos_set > motor_info[i].range) ? motor_info[i].range : motor_info[i].ctrl.pos_set;
		motor_info[i].ctrl.pos_set = (motor_info[i].ctrl.pos_set < -motor_info[i].range) ? -motor_info[i].range : motor_info[i].ctrl.pos_set;

		if (motor_info[i].para.pos > motor_info[i].range)
		{
			motor_info[i].ctrl.pos_set = motor_info[i].range - 0.1f;
			motor_info[i].ctrl.vel_set = 0;
			motor_info[i].ctrl.tor_set = 0;
			motor_info[i].ctrl.kp_set = 10;
			motor_info[i].ctrl.kd_set = 1;
		}
		else if (motor_info[i].para.pos < -motor_info[i].range)
		{
			motor_info[i].ctrl.pos_set = -motor_info[i].range + 0.1f;
			motor_info[i].ctrl.vel_set = 0;
			motor_info[i].ctrl.tor_set = 0;
			motor_info[i].ctrl.kp_set = 10;
			motor_info[i].ctrl.kd_set = 1;
		}
		else 
		{
			// motor_info[i].ctrl.kp_set = kps[i];
			// motor_info[i].ctrl.kd_set = kds[i];	
			motor_info[i].ctrl.kp_set = 0.0f;
			motor_info[i].ctrl.kd_set = 0.0f;
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
	dm4310_ctrl_send(&motor_info[Left_Ankle_Pitch]);
	dm4310_ctrl_send(&motor_info[Right_Hip_Yaw]);
	delay_us(200);
	dm4310_ctrl_send(&motor_info[Right_Hip_Roll]);
	dm4310_ctrl_send(&motor_info[Right_Hip_Pitch]);
	delay_us(200);
	dm4310_ctrl_send(&motor_info[Right_Knee_Pitch]);
	dm4310_ctrl_send(&motor_info[Right_Ankle_Pitch]);
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
	dm4310_disable(&motor_info[Left_Ankle_Pitch]);
	dm4310_disable(&motor_info[Right_Hip_Yaw]);
	delay_us(200);
	dm4310_disable(&motor_info[Right_Hip_Roll]);
	dm4310_disable(&motor_info[Right_Hip_Pitch]);
	delay_us(200);
	dm4310_disable(&motor_info[Right_Knee_Pitch]);
	dm4310_disable(&motor_info[Right_Ankle_Pitch]);
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
	dm4310_enable(&motor_info[Left_Ankle_Pitch]);
	dm4310_enable(&motor_info[Right_Hip_Yaw]);
	delay_us(200);
	dm4310_enable(&motor_info[Right_Hip_Roll]);
	dm4310_enable(&motor_info[Right_Hip_Pitch]);
	delay_us(200);
	dm4310_enable(&motor_info[Right_Knee_Pitch]);
	dm4310_enable(&motor_info[Right_Ankle_Pitch]);
	delay_us(200);
}
