#ifndef __DM4310_CTRL_H__
#define __DM4310_CTRL_H__
#include "main.h"
#include "DM4310driver.h"

extern int8_t motor_id;

extern motor_t motor[num];

void dm4310_motor_init(void);
void ctrl_enable(void);
void ctrl_disable(void);
void ctrl_set(void);
void ctrl_clear_para(void);
void ctrl_clear_err(void);
void ctrl_add(void);
void ctrl_minus(void);
void ctrl_send(void);

#endif /* __DM4310_CTRL_H__ */

