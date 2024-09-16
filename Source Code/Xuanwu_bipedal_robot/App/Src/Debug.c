#include "main.h"
#include "cmsis_os.h"
#include "usart.h"
#include <stdio.h>
#include "IMU.h"
#include "Robot.h"

extern IMU_t g_IMU;

void Debug_Task(void const * argument)
{
	portTickType xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();
  const TickType_t TimeIncrement = pdMS_TO_TICKS(100);
	
	HAL_UART_Init(&huart7);
	for(;;)
  {
		printf("/*%f,%f,%f,%f,%f,%f*/\n",
		g_Robot.left_foot.x,g_Robot.left_foot.y,g_Robot.left_foot.z,
		g_Robot.right_foot.x,g_Robot.right_foot.y,g_Robot.right_foot.z);
    vTaskDelayUntil(&xLastWakeTime, TimeIncrement);
  }
}

#pragma import(__use_no_semihosting)     
 
int _ttywrch(int ch)   
{
    ch=ch;
    return ch;
}
            
struct __FILE
{
    int handle;
    /* Whatever you require here. If the only file you are using is */
    /* standard output using printf() for debugging, no file handling */
    /* is required. */
};
 
FILE __stdout;    
  
void _sys_exit(int x)
{
    x = x;
}
 
int fputc(int ch, FILE *f){     
    HAL_UART_Transmit(&huart7,(uint8_t*)&ch,1,0xFFFF); 
		return ch;
}
