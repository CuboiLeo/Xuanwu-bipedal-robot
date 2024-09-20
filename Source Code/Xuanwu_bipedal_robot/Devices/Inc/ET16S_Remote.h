#ifndef ET16S_Remote_H
#define ET16S_Remote_H

#ifdef __cplusplus
extern "C" {
#endif

#define REMOTE_UART huart5
#define REMOTE_BUFFER_SIZE 25

#include "usart.h"
#include <stdint.h>

typedef struct Remote
{
    uint8_t remote_buffer[REMOTE_BUFFER_SIZE];
    int16_t val[16];
}Remote_t;

extern Remote_t g_Remote;

void Remote_Init(UART_HandleTypeDef *huart);

#ifdef __cplusplus
}
#endif

#endif
