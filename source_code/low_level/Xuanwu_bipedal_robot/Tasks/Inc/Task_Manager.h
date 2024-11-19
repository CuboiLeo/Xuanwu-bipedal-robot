#ifndef TASK_MANAGER_H
#define TASK_MANAGER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "cmsis_os.h"
#include "tim.h"

extern void Debug_Task(void *argument);
extern void IMU_Task(void *argument);
extern void Robot_Task(void *argument);
extern void Motor_Ctrl_Task(void *argument);
extern void System_Monitor_Task(void *argument);
exterm void Orin_Task(void *argument);

#ifdef __cplusplus
}
#endif

#endif