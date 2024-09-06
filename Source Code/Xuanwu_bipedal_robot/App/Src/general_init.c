#include "main.h"
#include "cmsis_os.h"

void Initialization_Task(void const * argument)
{
	for(;;)
  {
    osDelay(10);
  }
}