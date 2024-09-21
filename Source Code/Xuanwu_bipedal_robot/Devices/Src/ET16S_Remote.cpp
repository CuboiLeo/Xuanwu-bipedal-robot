#include "ET16S_Remote.h"
#include "dma.h"
#include "usart.h"

Remote::Remote()
{
    for(uint8_t i = 0; i < REMOTE_CHANNEL_NUM; i++)
    {
        channel_val[i] = 0;
    }
}

void Remote::Init(UART_HandleTypeDef *huart)
{
    HAL_UART_Init(huart);
    HAL_UARTEx_ReceiveToIdle_DMA(huart, remote_buffer, REMOTE_BUFFER_SIZE); // enable uart receive
    __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT); // disable half transfer interrupt
}

void Remote::processBuffer(void)
{
    if(remote_buffer[0] == 0x0F)
    {
        channel_val[0] = ((remote_buffer[1] | remote_buffer[2] << 8) & 0x07FF) - 1024; 
        channel_val[1] = ((remote_buffer[2] >> 3 | remote_buffer[3] << 5) & 0x07FF) - 1024;
        channel_val[2] = ((remote_buffer[3] >> 6 | remote_buffer[4] << 2 | remote_buffer[5] << 10) & 0x07FF) - 1024;
        channel_val[3] = ((remote_buffer[5] >> 1 | remote_buffer[6] << 7) & 0x07FF) - 1024;
        channel_val[4] = ((remote_buffer[6] >> 4 | remote_buffer[7] << 4) & 0x07FF) - 1024;
        channel_val[5] = ((remote_buffer[7] >> 7 | remote_buffer[8] << 1 | remote_buffer[9] << 9) & 0x07FF) - 1024;
        channel_val[6] = ((remote_buffer[9] >> 2 | remote_buffer[10] << 6) & 0x07FF) - 1024;
        channel_val[7] = ((remote_buffer[10] >> 5 | remote_buffer[11] << 3) & 0x07FF) - 1024;
        channel_val[8] = ((remote_buffer[12] | remote_buffer[13] << 8) & 0x07FF) - 1024;
        channel_val[9] = ((remote_buffer[13] >> 3 | remote_buffer[14] << 5) & 0x07FF) - 1024;
        channel_val[10] = ((remote_buffer[14] >> 6 | remote_buffer[15] << 2 | remote_buffer[16] << 10) & 0x07FF) - 1024;
        channel_val[11] = ((remote_buffer[16] >> 1 | remote_buffer[17] << 7) & 0x07FF) - 1024;
        channel_val[12] = ((remote_buffer[17] >> 4 | remote_buffer[18] << 4) & 0x07FF) - 1024;
        channel_val[13] = ((remote_buffer[18] >> 7 | remote_buffer[19] << 1 | remote_buffer[20] << 9) & 0x07FF) - 1024;
        channel_val[14] = ((remote_buffer[20] >> 2 | remote_buffer[21] << 6) & 0x07FF) - 1024;
        channel_val[15] = ((remote_buffer[21] >> 5 | remote_buffer[22] << 3) & 0x07FF) - 1024;
    }
}
