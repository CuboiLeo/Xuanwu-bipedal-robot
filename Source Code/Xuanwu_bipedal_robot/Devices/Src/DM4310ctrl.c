#include "DM4310driver.h"
#include "DM4310ctrl.h"
#include "string.h"
#include "Delay.h"

motor_t g_Motor[joint_num];

void dm4310_motor_init(void)
{
	memset(&g_Motor[Left_Hip_Yaw], 0, sizeof(g_Motor[Left_Hip_Yaw]));
	memset(&g_Motor[Left_Hip_Roll], 0, sizeof(g_Motor[Left_Hip_Roll]));
	memset(&g_Motor[Left_Hip_Pitch], 0, sizeof(g_Motor[Left_Hip_Pitch]));
	memset(&g_Motor[Left_Knee_Pitch], 0, sizeof(g_Motor[Left_Knee_Pitch]));
	memset(&g_Motor[Right_Hip_Yaw], 0, sizeof(g_Motor[Right_Hip_Yaw]));
	memset(&g_Motor[Right_Hip_Roll], 0, sizeof(g_Motor[Right_Hip_Roll]));
	memset(&g_Motor[Right_Hip_Pitch], 0, sizeof(g_Motor[Right_Hip_Pitch]));
	memset(&g_Motor[Right_Knee_Pitch], 0, sizeof(g_Motor[Right_Knee_Pitch]));

	g_Motor[Left_Hip_Yaw].can_bus = 1;
	g_Motor[Left_Hip_Yaw].id = 0x01;
	g_Motor[Left_Hip_Yaw].ctrl.mode = 0; // 0: MIT, 1: Pos, 2: Vel
	g_Motor[Left_Hip_Yaw].range = 1.5f;

	g_Motor[Left_Hip_Roll].can_bus = 1;
	g_Motor[Left_Hip_Roll].id = 0x02;
	g_Motor[Left_Hip_Roll].ctrl.mode = 0;
	g_Motor[Left_Hip_Roll].range = 0.2f;

	g_Motor[Left_Hip_Pitch].can_bus = 1;
	g_Motor[Left_Hip_Pitch].id = 0x03;
	g_Motor[Left_Hip_Pitch].ctrl.mode = 0;
	g_Motor[Left_Hip_Pitch].range = 0.6f;

	g_Motor[Left_Knee_Pitch].can_bus = 1;
	g_Motor[Left_Knee_Pitch].id = 0x04;
	g_Motor[Left_Knee_Pitch].ctrl.mode = 0;
	g_Motor[Left_Knee_Pitch].range = 0.6f;

	g_Motor[Right_Hip_Yaw].can_bus = 2;
	g_Motor[Right_Hip_Yaw].id = 0x05;
	g_Motor[Right_Hip_Yaw].ctrl.mode = 0;
	g_Motor[Right_Hip_Yaw].range = 1.5f;

	g_Motor[Right_Hip_Roll].can_bus = 2;
	g_Motor[Right_Hip_Roll].id = 0x06;
	g_Motor[Right_Hip_Roll].ctrl.mode = 0;
	g_Motor[Right_Hip_Roll].range = 0.2f;

	g_Motor[Right_Hip_Pitch].can_bus = 2;
	g_Motor[Right_Hip_Pitch].id = 0x07;
	g_Motor[Right_Hip_Pitch].ctrl.mode = 0;
	g_Motor[Right_Hip_Pitch].range = 0.6f;

	g_Motor[Right_Knee_Pitch].can_bus = 2;
	g_Motor[Right_Knee_Pitch].id = 0x08;
	g_Motor[Right_Knee_Pitch].ctrl.mode = 0;
	g_Motor[Right_Knee_Pitch].range = 0.6f;
}

void fdcan1_rx_callback(void)
{
	uint16_t rec_id;
	uint8_t rx_data[8] = {0};
	fdcanx_receive(&hfdcan1, &rec_id, rx_data);
	switch (rec_id)
	{
 		case 0x11: dm4310_fbdata(&g_Motor[Left_Hip_Yaw], rx_data); break;
		case 0x12: dm4310_fbdata(&g_Motor[Left_Hip_Roll], rx_data); break;
		case 0x13: dm4310_fbdata(&g_Motor[Left_Hip_Pitch], rx_data); break;
		case 0x14: dm4310_fbdata(&g_Motor[Left_Knee_Pitch], rx_data); break;
	}
}

void fdcan2_rx_callback(void)
{
	uint16_t rec_id;
	uint8_t rx_data[8] = {0};
	fdcanx_receive(&hfdcan2, &rec_id, rx_data);
	switch (rec_id)
	{
		case 0x15: dm4310_fbdata(&g_Motor[Right_Hip_Yaw], rx_data); break;
		case 0x16: dm4310_fbdata(&g_Motor[Right_Hip_Roll], rx_data); break;
		case 0x17: dm4310_fbdata(&g_Motor[Right_Hip_Pitch], rx_data); break;
		case 0x18: dm4310_fbdata(&g_Motor[Right_Knee_Pitch], rx_data); break;
	}
}

void dm4310_reset_joints(void)
{
	dm4310_disable_all();
	for (int i = 0; i < joint_num; i++)
	{
		dm4310_clear_para(&g_Motor[i]);
	}
	dm4310_enable_all();
}

uint8_t dm4310_return_zero_pos(void)
{
	uint8_t all_joints_reached_pos_flag = 1;
	for(int i = 0; i < joint_num; i++)
	{
		g_Motor[i].ctrl.pos_set = 0;
		g_Motor[i].ctrl.kp_set = 2;
		g_Motor[i].ctrl.kd_set = 1;

		if(fabs(g_Motor[i].para.pos - g_Motor[i].ctrl.pos_set) < 0.2f)
		{
			g_Motor[i].reached_pos_flag = 1;
		}
		else
		{
			g_Motor[i].reached_pos_flag = 0;
		}

		all_joints_reached_pos_flag = all_joints_reached_pos_flag && g_Motor[i].reached_pos_flag;
	}

	if (all_joints_reached_pos_flag)
	{
		for(int i = 0; i < joint_num; i++)
		{
			g_Motor[i].ctrl.pos_set = 0;
			g_Motor[i].ctrl.kp_set = 0;
			g_Motor[i].ctrl.kd_set = 0;
		}
	}
	
	return all_joints_reached_pos_flag;
}

void dm4310_send_all(void)
{
	dm4310_ctrl_send(&g_Motor[Left_Hip_Yaw]);
	dm4310_ctrl_send(&g_Motor[Left_Hip_Roll]);
	delay_us(200);
	dm4310_ctrl_send(&g_Motor[Left_Hip_Pitch]);
	dm4310_ctrl_send(&g_Motor[Left_Knee_Pitch]);
	delay_us(200);
	dm4310_ctrl_send(&g_Motor[Right_Hip_Yaw]);
	dm4310_ctrl_send(&g_Motor[Right_Hip_Roll]);
	delay_us(200);
	dm4310_ctrl_send(&g_Motor[Right_Hip_Pitch]);
	dm4310_ctrl_send(&g_Motor[Right_Knee_Pitch]);
	delay_us(200);
}

void dm4310_disable_all(void)
{
	dm4310_disable(&g_Motor[Left_Hip_Yaw]);
	dm4310_disable(&g_Motor[Left_Hip_Roll]);
	delay_us(200);
	dm4310_disable(&g_Motor[Left_Hip_Pitch]);
	dm4310_disable(&g_Motor[Left_Knee_Pitch]);
	delay_us(200);
	dm4310_disable(&g_Motor[Right_Hip_Yaw]);
	dm4310_disable(&g_Motor[Right_Hip_Roll]);
	delay_us(200);
	dm4310_disable(&g_Motor[Right_Hip_Pitch]);
	dm4310_disable(&g_Motor[Right_Knee_Pitch]);
	delay_us(200);
}

void dm4310_enable_all(void)
{
	dm4310_enable(&g_Motor[Left_Hip_Yaw]);
	dm4310_enable(&g_Motor[Left_Hip_Roll]);
	delay_us(200);
	dm4310_enable(&g_Motor[Left_Hip_Pitch]);
	dm4310_enable(&g_Motor[Left_Knee_Pitch]);
	delay_us(200);
	dm4310_enable(&g_Motor[Right_Hip_Yaw]);
	dm4310_enable(&g_Motor[Right_Hip_Roll]);
	delay_us(200);
	dm4310_enable(&g_Motor[Right_Hip_Pitch]);
	dm4310_enable(&g_Motor[Right_Knee_Pitch]);
	delay_us(200);
}

void dm4310_virtual_boundary(void)
{
	for (int i = 0; i < joint_num; i++)
	{
		g_Motor[i].ctrl.pos_set = (g_Motor[i].ctrl.pos_set > g_Motor[i].range) ? g_Motor[i].range : g_Motor[i].ctrl.pos_set;
		g_Motor[i].ctrl.pos_set = (g_Motor[i].ctrl.pos_set < -g_Motor[i].range) ? -g_Motor[i].range : g_Motor[i].ctrl.pos_set;
			
		if(g_Motor[i].para.pos > g_Motor[i].range)
		{
			g_Motor[i].ctrl.pos_set = g_Motor[i].range - 0.1f;
			g_Motor[i].ctrl.vel_set = 0;
			g_Motor[i].ctrl.tor_set = 0;
			g_Motor[i].ctrl.kp_set = 3;
			g_Motor[i].ctrl.kd_set = 1;
		}
		else if(g_Motor[i].para.pos < -g_Motor[i].range)
		{
			g_Motor[i].ctrl.pos_set = -g_Motor[i].range + 0.1f;
			g_Motor[i].ctrl.vel_set = 0;
			g_Motor[i].ctrl.tor_set = 0;
			g_Motor[i].ctrl.kp_set = 3;
			g_Motor[i].ctrl.kd_set = 1;
		}
	}
}
