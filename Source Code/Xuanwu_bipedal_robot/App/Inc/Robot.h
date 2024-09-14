#ifndef ROBOT_H
#define ROBOT_H

#include "stdint.h"

typedef struct Robot
{
    uint8_t soft_start_flag;
} Robot_t;

extern Robot_t g_Robot;
#endif
