#include "DM4310driver.h"
#include "fdcan.h"

void dm4310_enable(motor_t *motor)
{
	FDCAN_HandleTypeDef *hcan;
	if(motor->can_bus == 1){
		hcan = &hfdcan1;
	}
	else if (motor->can_bus == 2)
	{
		hcan = &hfdcan2;
	}
	
	switch(motor->ctrl.mode)
	{
		case 0:
			enable_motor_mode(hcan, motor->id, MIT_MODE);
			break;
		case 1:
			enable_motor_mode(hcan, motor->id, POS_MODE);
			break;
		case 2:
			enable_motor_mode(hcan, motor->id, SPEED_MODE);
			break;
		case 3:
			enable_motor_mode(hcan, motor->id, POSI_MODE);
			break;
	}	
}

void dm4310_disable(motor_t *motor)
{
	FDCAN_HandleTypeDef *hcan;
	if(motor->can_bus == 1){
		hcan = &hfdcan1;
	}
	else if (motor->can_bus == 2)
	{
		hcan = &hfdcan2;
	}

	switch(motor->ctrl.mode)
	{
		case 0:
			disable_motor_mode(hcan, motor->id, MIT_MODE);
			break;
		case 1:
			disable_motor_mode(hcan, motor->id, POS_MODE);
			break;
		case 2:
			disable_motor_mode(hcan, motor->id, SPEED_MODE);
			break;
		case 3:
			disable_motor_mode(hcan, motor->id, POSI_MODE);
			break;
	}	
	dm4310_clear_para(motor);
}

void dm4310_ctrl_send(motor_t *motor)
{
	FDCAN_HandleTypeDef *hcan;
	if(motor->can_bus == 1){
		hcan = &hfdcan1;
	}
	else if (motor->can_bus == 2)
	{
		hcan = &hfdcan2;
	}
	
	switch(motor->ctrl.mode)
	{
		case 0:
			mit_ctrl(hcan, motor->id, motor->ctrl.pos_set, motor->ctrl.vel_set, motor->ctrl.kp_set, motor->ctrl.kd_set, motor->ctrl.tor_set);
			break;
		case 1:
			pos_speed_ctrl(hcan, motor->id, motor->ctrl.pos_set, motor->ctrl.vel_set);
			break;
		case 2:
			speed_ctrl(hcan, motor->id, motor->ctrl.vel_set);
			break;
		case 3:
			pos_force_ctrl(hcan, motor->id,motor->ctrl.pos_set, motor->ctrl.vel_set, motor->ctrl.tor_set);
			break;
	}	
}

void dm4310_clear_para(motor_t *motor)
{
	motor->ctrl.kd_set 	= 0;
	motor->ctrl.kp_set	= 0;
	motor->ctrl.pos_set = 0;
	motor->ctrl.vel_set = 0;
	motor->ctrl.tor_set = 0;
}

void dm4310_clear_err(motor_t *motor)
{
	FDCAN_HandleTypeDef *hcan;
	if(motor->can_bus == 1){
		hcan = &hfdcan1;
	}
	else if (motor->can_bus == 2)
	{
		hcan = &hfdcan2;
	}

	switch(motor->ctrl.mode)
	{
		case 0:
			clear_err(hcan, motor->id, MIT_MODE);
			break;
		case 1:
			clear_err(hcan, motor->id, POS_MODE);
			break;
		case 2:
			clear_err(hcan, motor->id, SPEED_MODE);
			break;
	}	
}

void dm4310_fbdata(motor_t *motor, uint8_t *rx_data)
{
	motor->para.id = (rx_data[0])&0x0F;
	motor->para.state = (rx_data[0])>>4;
	motor->para.p_int=(rx_data[1]<<8)|rx_data[2];
	motor->para.v_int=(rx_data[3]<<4)|(rx_data[4]>>4);
	motor->para.t_int=((rx_data[4]&0xF)<<8)|rx_data[5];
	motor->para.pos = uint_to_float(motor->para.p_int, P_MIN, P_MAX, 16); // (-12.5,12.5)
	motor->para.vel = (1.0f - LPF_COEFF)*motor->para.vel + LPF_COEFF*uint_to_float(motor->para.v_int, V_MIN, V_MAX, 12); // (-45.0,45.0)
	motor->para.tor = uint_to_float(motor->para.t_int, T_MIN, T_MAX, 12);  // (-18.0,18.0)
	motor->para.Tmos = (float)(rx_data[6]);
	motor->para.Tcoil = (float)(rx_data[7]);
}

int float_to_uint(float x_float, float x_min, float x_max, int bits)
{
	/* Converts a float to an unsigned int, given range and number of bits */
	float span = x_max - x_min;
	float offset = x_min;
	return (int) ((x_float-offset)*((float)((1<<bits)-1))/span);
}

float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
	/* converts unsigned int to float, given range and number of bits */
	float span = x_max - x_min;
	float offset = x_min;
	return ((float)x_int)*span/((float)((1<<bits)-1)) + offset;
}

void enable_motor_mode(FDCAN_HandleTypeDef *hcan, uint16_t motor_id, uint16_t mode_id)
{
	uint8_t data[8];
	uint16_t id = motor_id + mode_id;
	
	data[0] = 0xFF;
	data[1] = 0xFF;
	data[2] = 0xFF;
	data[3] = 0xFF;
	data[4] = 0xFF;
	data[5] = 0xFF;
	data[6] = 0xFF;
	data[7] = 0xFC;
	
	fdcanx_send_data(hcan, id, data, 8);
}

void disable_motor_mode(FDCAN_HandleTypeDef *hcan, uint16_t motor_id, uint16_t mode_id)
{
	uint8_t data[8];
	uint16_t id = motor_id + mode_id;
	
	data[0] = 0xFF;
	data[1] = 0xFF;
	data[2] = 0xFF;
	data[3] = 0xFF;
	data[4] = 0xFF;
	data[5] = 0xFF;
	data[6] = 0xFF;
	data[7] = 0xFD;
	
	fdcanx_send_data(hcan, id, data, 8);
}

void save_pos_zero(FDCAN_HandleTypeDef *hcan, uint16_t motor_id, uint16_t mode_id)
{
	uint8_t data[8];
	uint16_t id = motor_id + mode_id;
	
	data[0] = 0xFF;
	data[1] = 0xFF;
	data[2] = 0xFF;
	data[3] = 0xFF;
	data[4] = 0xFF;
	data[5] = 0xFF;
	data[6] = 0xFF;
	data[7] = 0xFE;
	
	fdcanx_send_data(hcan, id, data, 8);
}

void clear_err(FDCAN_HandleTypeDef *hcan, uint16_t motor_id, uint16_t mode_id)
{
	uint8_t data[8];
	uint16_t id = motor_id + mode_id;
	
	data[0] = 0xFF;
	data[1] = 0xFF;
	data[2] = 0xFF;
	data[3] = 0xFF;
	data[4] = 0xFF;
	data[5] = 0xFF;
	data[6] = 0xFF;
	data[7] = 0xFB;
	
	fdcanx_send_data(hcan, id, data, 8);
}

void mit_ctrl(FDCAN_HandleTypeDef *hcan, uint16_t motor_id, float pos, float vel,float kp, float kd, float torq)
{
	uint8_t data[8];
	uint16_t pos_tmp,vel_tmp,kp_tmp,kd_tmp,tor_tmp;
	uint16_t id = motor_id + MIT_MODE;

	pos_tmp = float_to_uint(pos,  P_MIN,  P_MAX,  16);
	vel_tmp = float_to_uint(vel,  V_MIN,  V_MAX,  12);
	kp_tmp  = float_to_uint(kp,   KP_MIN, KP_MAX, 12);
	kd_tmp  = float_to_uint(kd,   KD_MIN, KD_MAX, 12);
	tor_tmp = float_to_uint(torq, T_MIN,  T_MAX,  12);

	data[0] = (pos_tmp >> 8);
	data[1] = pos_tmp;
	data[2] = (vel_tmp >> 4);
	data[3] = ((vel_tmp&0xF)<<4)|(kp_tmp>>8);
	data[4] = kp_tmp;
	data[5] = (kd_tmp >> 4);
	data[6] = ((kd_tmp&0xF)<<4)|(tor_tmp>>8);
	data[7] = tor_tmp;
	
	fdcanx_send_data(hcan, id, data, 8);
}

void pos_speed_ctrl(FDCAN_HandleTypeDef *hcan,uint16_t motor_id, float pos, float vel)
{
	uint16_t id;
	uint8_t *pbuf, *vbuf;
	uint8_t data[8];
	
	id = motor_id + POS_MODE;
	pbuf=(uint8_t*)&pos;
	vbuf=(uint8_t*)&vel;
	
	data[0] = *pbuf;
	data[1] = *(pbuf+1);
	data[2] = *(pbuf+2);
	data[3] = *(pbuf+3);

	data[4] = *vbuf;
	data[5] = *(vbuf+1);
	data[6] = *(vbuf+2);
	data[7] = *(vbuf+3);
	
	fdcanx_send_data(hcan, id, data, 8);
}

void speed_ctrl(FDCAN_HandleTypeDef *hcan,uint16_t motor_id, float vel)
{
	uint16_t id;
	uint8_t *vbuf;
	uint8_t data[4];
	
	id = motor_id + SPEED_MODE;
	vbuf=(uint8_t*)&vel;
	
	data[0] = *vbuf;
	data[1] = *(vbuf+1);
	data[2] = *(vbuf+2);
	data[3] = *(vbuf+3);
	
	fdcanx_send_data(hcan, id, data, 4);
}

void pos_force_ctrl(FDCAN_HandleTypeDef *hcan,uint16_t motor_id, float pos, uint16_t vel, uint16_t i)
{
	uint16_t id;
	uint8_t *pbuf, *vbuf, *ibuf;
	uint8_t data[8];
	
	id = motor_id + POSI_MODE;
	pbuf=(uint8_t*)&pos;
	vbuf=(uint8_t*)&vel;
	ibuf=(uint8_t*)&i;
	
	data[0] = *pbuf;
	data[1] = *(pbuf+1);
	data[2] = *(pbuf+2);
	data[3] = *(pbuf+3);

	data[4] = *vbuf;
	data[5] = *(vbuf+1);
	
	data[6] = *ibuf;
	data[7] = *(ibuf+1);
	
	fdcanx_send_data(hcan, id, data, 8);
}
