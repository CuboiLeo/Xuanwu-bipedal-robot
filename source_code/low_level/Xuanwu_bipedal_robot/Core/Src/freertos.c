/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "Task_Manager.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
typedef StaticTask_t osStaticThreadDef_t;
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
osSemaphoreId imuBinarySem01Handle;
StaticSemaphore_t  imuBinarySemControlBlock;
/* USER CODE END Variables */
/* Definitions for Debug */
osThreadId_t DebugHandle;
const osThreadAttr_t Debug_attributes = {
  .name = "Debug",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for IMU */
osThreadId_t IMUHandle;
uint32_t IMUBuffer[ 512 ];
osStaticThreadDef_t IMUControlBlock;
const osThreadAttr_t IMU_attributes = {
  .name = "IMU",
  .cb_mem = &IMUControlBlock,
  .cb_size = sizeof(IMUControlBlock),
  .stack_mem = &IMUBuffer[0],
  .stack_size = sizeof(IMUBuffer),
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for Robot */
osThreadId_t RobotHandle;
const osThreadAttr_t Robot_attributes = {
  .name = "Robot",
  .stack_size = 1280 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for Motor_Ctrl */
osThreadId_t Motor_CtrlHandle;
const osThreadAttr_t Motor_Ctrl_attributes = {
  .name = "Motor_Ctrl",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for System_Monitor */
osThreadId_t System_MonitorHandle;
const osThreadAttr_t System_Monitor_attributes = {
  .name = "System_Monitor",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void Debug_Task(void *argument);
void IMU_Task(void *argument);
void Robot_Task(void *argument);
void Motor_Ctrl_Task(void *argument);
void System_Monitor_Task(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* Hook prototypes */
void configureTimerForRunTimeStats(void);
unsigned long getRunTimeCounterValue(void);

/* USER CODE BEGIN 1 */
/* Functions needed when configGENERATE_RUN_TIME_STATS is on */
__weak void configureTimerForRunTimeStats(void)
{

}

__weak unsigned long getRunTimeCounterValue(void)
{
return 0;
}
/* USER CODE END 1 */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* definition and creation of imuBinarySem01 */
  const osSemaphoreAttr_t imuBinarySem01_attributes = {
    .name = "imuBinarySem01",
    .cb_mem = &imuBinarySemControlBlock,       // Pointer to the static control block
    .cb_size = sizeof(imuBinarySemControlBlock) // Size of the control block
	};
	osSemaphoreId_t imuBinarySem01Handle = osSemaphoreNew(1, 1, &imuBinarySem01_attributes);
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of Debug */
  DebugHandle = osThreadNew(Debug_Task, NULL, &Debug_attributes);

  /* creation of IMU */
  IMUHandle = osThreadNew(IMU_Task, NULL, &IMU_attributes);

  /* creation of Robot */
  RobotHandle = osThreadNew(Robot_Task, NULL, &Robot_attributes);

  /* creation of Motor_Ctrl */
  Motor_CtrlHandle = osThreadNew(Motor_Ctrl_Task, NULL, &Motor_Ctrl_attributes);

  /* creation of System_Monitor */
  System_MonitorHandle = osThreadNew(System_Monitor_Task, NULL, &System_Monitor_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_Debug_Task */
/**
  * @brief  Function implementing the Debug thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_Debug_Task */
__weak void Debug_Task(void *argument)
{
  /* USER CODE BEGIN Debug_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Debug_Task */
}

/* USER CODE BEGIN Header_IMU_Task */
/**
* @brief Function implementing the IMU thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_IMU_Task */
__weak void IMU_Task(void *argument)
{
  /* USER CODE BEGIN IMU_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END IMU_Task */
}

/* USER CODE BEGIN Header_Robot_Task */
/**
* @brief Function implementing the Robot thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Robot_Task */
__weak void Robot_Task(void *argument)
{
  /* USER CODE BEGIN Robot_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Robot_Task */
}

/* USER CODE BEGIN Header_Motor_Ctrl_Task */
/**
* @brief Function implementing the Motor_Ctrl thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Motor_Ctrl_Task */
__weak void Motor_Ctrl_Task(void *argument)
{
  /* USER CODE BEGIN Motor_Ctrl_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Motor_Ctrl_Task */
}

/* USER CODE BEGIN Header_System_Monitor_Task */
/**
* @brief Function implementing the System_Monitor thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_System_Monitor_Task */
__weak void System_Monitor_Task(void *argument)
{
  /* USER CODE BEGIN System_Monitor_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END System_Monitor_Task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

