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
