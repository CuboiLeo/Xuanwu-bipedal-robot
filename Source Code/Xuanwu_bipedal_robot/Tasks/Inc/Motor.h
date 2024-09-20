#ifndef MOTOR_H
#define MOTOR_H

#ifdef __cplusplus
extern "C" {
#endif

#include "DM4310ctrl.h"
#define	FDCAN1_PowerUp(x) HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, x)
#define	FDCAN2_PowerUp(x) HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, x)
#define ALL_JOINTS_ZEROED (1)
	
extern void Motor_Ctrl_Task(void const * argument);

#ifdef __cplusplus
}
#endif

#endif
