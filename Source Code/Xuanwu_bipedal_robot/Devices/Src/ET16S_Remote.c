#include "ET16S_Remote.h"
#include "dma.h"
#include "usart.h"

Remote_t g_Remote;

void Remote_Init(UART_HandleTypeDef *huart)
{
    HAL_UART_Init(huart);
    // enable uart receive
    HAL_UARTEx_ReceiveToIdle_DMA(huart, g_Remote.remote_buffer, REMOTE_BUFFER_SIZE);
    // disable half transfer interrupt
    __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT); // disable half transfer interrupt
}

void Remote_Buffer_Process(void)
{
    // process remote data
    if(g_Remote.remote_buffer[0] == 0x0F)
    {
        g_Remote.val[0] = ((g_Remote.remote_buffer[1] | g_Remote.remote_buffer[2] << 8) & 0x07FF) - 1024; 
        g_Remote.val[1] = ((g_Remote.remote_buffer[2] >> 3 | g_Remote.remote_buffer[3] << 5) & 0x07FF) - 1024;
        g_Remote.val[2] = ((g_Remote.remote_buffer[3] >> 6 | g_Remote.remote_buffer[4] << 2 | g_Remote.remote_buffer[5] << 10) & 0x07FF) - 1024;
        g_Remote.val[3] = ((g_Remote.remote_buffer[5] >> 1 | g_Remote.remote_buffer[6] << 7) & 0x07FF) - 1024;
        g_Remote.val[4] = ((g_Remote.remote_buffer[6] >> 4 | g_Remote.remote_buffer[7] << 4) & 0x07FF) - 1024;
        g_Remote.val[5] = ((g_Remote.remote_buffer[7] >> 7 | g_Remote.remote_buffer[8] << 1 | g_Remote.remote_buffer[9] << 9) & 0x07FF) - 1024;
        g_Remote.val[6] = ((g_Remote.remote_buffer[9] >> 2 | g_Remote.remote_buffer[10] << 6) & 0x07FF) - 1024;
        g_Remote.val[7] = ((g_Remote.remote_buffer[10] >> 5 | g_Remote.remote_buffer[11] << 3) & 0x07FF) - 1024;
        g_Remote.val[8] = ((g_Remote.remote_buffer[12] | g_Remote.remote_buffer[13] << 8) & 0x07FF) - 1024;
        g_Remote.val[9] = ((g_Remote.remote_buffer[13] >> 3 | g_Remote.remote_buffer[14] << 5) & 0x07FF) - 1024;
        g_Remote.val[10] = ((g_Remote.remote_buffer[14] >> 6 | g_Remote.remote_buffer[15] << 2 | g_Remote.remote_buffer[16] << 10) & 0x07FF) - 1024;
        g_Remote.val[11] = ((g_Remote.remote_buffer[16] >> 1 | g_Remote.remote_buffer[17] << 7) & 0x07FF) - 1024;
        g_Remote.val[12] = ((g_Remote.remote_buffer[17] >> 4 | g_Remote.remote_buffer[18] << 4) & 0x07FF) - 1024;
        g_Remote.val[13] = ((g_Remote.remote_buffer[18] >> 7 | g_Remote.remote_buffer[19] << 1 | g_Remote.remote_buffer[20] << 9) & 0x07FF) - 1024;
        g_Remote.val[14] = ((g_Remote.remote_buffer[20] >> 2 | g_Remote.remote_buffer[21] << 6) & 0x07FF) - 1024;
        g_Remote.val[15] = ((g_Remote.remote_buffer[21] >> 5 | g_Remote.remote_buffer[22] << 3) & 0x07FF) - 1024;
    }
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if(huart->Instance == REMOTE_UART.Instance)
    {
        Remote_Buffer_Process();
        // enable uart receive for next data frame
        HAL_UARTEx_ReceiveToIdle_DMA(huart, g_Remote.remote_buffer, REMOTE_BUFFER_SIZE);
        // still disable half transfer interrupt (@ref void UART_Service_Init(void))
        __HAL_DMA_DISABLE_IT(REMOTE_UART.hdmarx, DMA_IT_HT);
    }
}
