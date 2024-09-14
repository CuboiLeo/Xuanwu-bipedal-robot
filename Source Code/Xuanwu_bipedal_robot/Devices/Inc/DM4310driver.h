#ifndef __DM4310_DRV_H__
#define __DM4310_DRV_H__
#include "main.h"
#include "fdcan.h"
#include "can_bsp.h"

#define MIT_MODE 			0x000
#define POS_MODE			0x100
#define SPEED_MODE		0x200
#define POSI_MODE		  0x300

#define P_MIN -12.5f
#define P_MAX 12.5f
#define V_MIN -30.0f
#define V_MAX 30.0f
#define KP_MIN 0.0f
#define KP_MAX 500.0f
#define KD_MIN 0.0f
#define KD_MAX 5.0f
#define T_MIN -10.0f
#define T_MAX 10.0f

typedef enum
{
	Left_Hip_Yaw,
	Left_Hip_Roll,
	Left_Hip_Pitch,
	Left_Knee_Pitch,
	Right_Hip_Yaw,
	Right_Hip_Roll,
	Right_Hip_Pitch,
	Right_Knee_Pitch,
	joint_num
} motor_num;

typedef struct 
{
	int id;
	int state;
	int p_int;
	int v_int;
	int t_int;
	int kp_int;
	int kd_int;
	float pos;
	float vel;
	float tor;
	float Kp;
	float Kd;
	float Tmos;
	float Tcoil;
}motor_fbpara_t;

typedef struct 
{
	int8_t mode;
	float pos_set;
	float vel_set;
	float tor_set;
	float kp_set;
	float kd_set;
}motor_ctrl_t;

typedef struct
{
	uint8_t can_bus;
	int16_t id;
	uint8_t reached_pos_flag;
	float range;
	motor_fbpara_t para;
	motor_ctrl_t ctrl;
}motor_t;

float uint_to_float(int x_int, float x_min, float x_max, int bits);
int float_to_uint(float x_float, float x_min, float x_max, int bits);
void dm4310_ctrl_send(motor_t *motor);
void dm4310_enable(motor_t *motor);
void dm4310_disable(motor_t *motor);
void dm4310_clear_para(motor_t *motor);
void dm4310_clear_err(motor_t *motor);
void dm4310_fbdata(motor_t *motor, uint8_t *rx_data);

void enable_motor_mode(FDCAN_HandleTypeDef *hcan, uint16_t motor_id, uint16_t mode_id);
void disable_motor_mode(FDCAN_HandleTypeDef *hcan, uint16_t motor_id, uint16_t mode_id);

void mit_ctrl(FDCAN_HandleTypeDef *hcan, uint16_t motor_id, float pos, float vel,float kp, float kd, float torq);
void pos_speed_ctrl(FDCAN_HandleTypeDef *hcan,uint16_t motor_id, float pos, float vel);
void speed_ctrl(FDCAN_HandleTypeDef *hcan,uint16_t motor_id, float _vel);
void pos_force_ctrl(FDCAN_HandleTypeDef *hcan,uint16_t motor_id, float pos, uint16_t vel, uint16_t i);

void save_pos_zero(FDCAN_HandleTypeDef *hcan, uint16_t motor_id, uint16_t mode_id);
void clear_err(FDCAN_HandleTypeDef *hcan, uint16_t motor_id, uint16_t mode_id);

#endif /* __DM4310_DRV_H__ */
