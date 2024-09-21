#include "main.h"
#include "cmsis_os.h"
#include "usart.h"
#include <cstdio>
#include "Robot.h"
#include "Debug.h"

extern "C" {
    __asm(".global __use_no_semihosting");  // for ARM Compiler 6

    // Function to write a character to UART (used by printf)
    int _ttywrch(int ch) {
        ch = ch;
        return ch;
    }

    // Placeholder for stdout; needed for printf
    FILE __stdout;

    // Prevent unused variable warning
    void _sys_exit(int x) {
        x = x;
    }

    // Redirect fputc to UART transmit for printf functionality
    int fputc(int ch, FILE *f) {
        HAL_UART_Transmit(&huart7, (uint8_t*)&ch, 1, 0xFFFF);
        return ch;
    }
}

void Debug_Task(void const * argument) {
    portTickType xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();
    const TickType_t TimeIncrement = pdMS_TO_TICKS(100);
    
    HAL_UART_Init(&huart7);

    for (;;) {
//         printf("/*%f,%f,%f,%f,%f,%f*/\n",
//                g_Robot.left_foot.x, g_Robot.left_foot.y, g_Robot.left_foot.z,
//                g_Robot.right_foot.x, g_Robot.right_foot.y, g_Robot.right_foot.z);

        vTaskDelayUntil(&xLastWakeTime, TimeIncrement);
    }
}
