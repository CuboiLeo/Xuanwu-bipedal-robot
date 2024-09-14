#ifndef __DM4310_CTRL_H__
#define __DM4310_CTRL_H__
#include "main.h"
#include "DM4310driver.h"

extern motor_t g_Motor[joint_num];

void dm4310_motor_init(void);
void dm4310_reset_joints(void);
void dm4310_send_all(void);
void dm4310_disable_all(void);
void dm4310_enable_all(void);
uint8_t dm4310_return_zero_pos(void);
void dm4310_virtual_boundary(void);

#endif /* __DM4310_CTRL_H__ */

