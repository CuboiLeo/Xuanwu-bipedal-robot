#ifndef ET16S_Remote_H
#define ET16S_Remote_H

#ifdef __cplusplus
extern "C" {
#endif

#define REMOTE_UART (huart5)

#include "usart.h"
#include <stdint.h>

class Remote
{
    public:
        Remote();
        static constexpr uint8_t REMOTE_BUFFER_SIZE = 25;
        static constexpr uint8_t REMOTE_CHANNEL_NUM = 16;
        void Init(UART_HandleTypeDef *huart);
        void processBuffer(void);

        friend void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size);
    private:
        int16_t channel_val[REMOTE_CHANNEL_NUM];
        uint8_t remote_buffer[REMOTE_BUFFER_SIZE];
};

#ifdef __cplusplus
}
#endif

#endif
