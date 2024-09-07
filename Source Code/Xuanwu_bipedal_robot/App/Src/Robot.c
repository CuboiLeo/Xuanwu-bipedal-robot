#include "ET16S_Remote.h"
#include "usart.h"
#include "cmsis_os.h"

void Robot_Task(void const * argument)
{
    portTickType xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();
    const TickType_t TimeIncrement = pdMS_TO_TICKS(100);

    Remote_Init(&REMOTE_UART);
    for(;;)
    {
        vTaskDelayUntil(&xLastWakeTime, TimeIncrement);
    }
}