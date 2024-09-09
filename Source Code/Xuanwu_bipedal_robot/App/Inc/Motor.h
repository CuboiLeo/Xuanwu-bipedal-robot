#include "DM4310ctrl.h"
#define	FDCAN1_PowerUp(x) HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, x)
#define	FDCAN2_PowerUp(x) HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, x)