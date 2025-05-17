/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "motor_task.h"
#include "usart_communicate.h"
#include "referee_usart_task.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
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

/* USER CODE END Variables */
osThreadId testHandle;
//osThreadId gimbalTaskHandle;
osThreadId imuTaskHandle;
//osThreadId VOFATaskHandle;
//osThreadId pcTaskHandle;


/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
osThreadId motorTaskHandle;
osThreadId uasrtcommunicateTaskHandle;
osThreadId refereecommunicateTaskHandle;
/* USER CODE END FunctionPrototypes */

void test_task(void const * argument);
void gimbal_task(void const * argument);
void ins_task(void const * argument);
void VOFA_task(void const * argument);
void pc_task(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

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
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of test */
//  osThreadDef(test, test_task, osPriorityHigh, 0, 128);
//  testHandle = osThreadCreate(osThread(test), NULL);

  /* definition and creation of gimbalTask */
//  osThreadDef(gimbalTask, gimbal_task, osPriorityHigh, 0, 512);
//  gimbalTaskHandle = osThreadCreate(osThread(gimbalTask), NULL);

  /* definition and creation of imuTask */
  osThreadDef(imuTask, ins_task, osPriorityNormal, 0, 1024);
  imuTaskHandle = osThreadCreate(osThread(imuTask), NULL);

  /* definition and creation of VOFATask */
//  osThreadDef(VOFATask, VOFA_task, osPriorityNormal, 0, 128);
//  VOFATaskHandle = osThreadCreate(osThread(VOFATask), NULL);

  /* definition and creation of pcTask */
//  osThreadDef(pcTask, pc_task, osPriorityNormal, 0, 256);
//  pcTaskHandle = osThreadCreate(osThread(pcTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
//	osThreadDef(motorTask, motor_task, osPriorityNormal, 0, 1024);
//  motorTaskHandle = osThreadCreate(osThread(motorTask), NULL);
	
	osThreadDef(uasrtcommunicateTask, usart_communicate_task, osPriorityNormal, 0, 256);
  uasrtcommunicateTaskHandle = osThreadCreate(osThread(uasrtcommunicateTask), NULL);
	
	osThreadDef(refereecommunicateTask, referee_usart_task, osPriorityNormal, 0, 256);
  refereecommunicateTaskHandle = osThreadCreate(osThread(refereecommunicateTask), NULL);

  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_test_task */
/**
  * @brief  Function implementing the test thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_test_task */
__weak void test_task(void const * argument)
{
  /* USER CODE BEGIN test_task */
  /* Infinite loop */
  for(;;)
  {
	
    osDelay(100);
  }
  /* USER CODE END test_task */
}

/* USER CODE BEGIN Header_gimbal_task */
/**
* @brief Function implementing the gimbalTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_gimbal_task */
__weak void gimbal_task(void const * argument)
{
  /* USER CODE BEGIN gimbal_task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END gimbal_task */
}

/* USER CODE BEGIN Header_ins_task */
/**
* @brief Function implementing the imuTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_ins_task */
__weak void ins_task(void const * argument)
{
  /* USER CODE BEGIN ins_task */

  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END ins_task */
}

/* USER CODE BEGIN Header_VOFA_task */
/**
* @brief Function implementing the VOFATask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_VOFA_task */
__weak void VOFA_task(void const * argument)
{
  /* USER CODE BEGIN VOFA_task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END VOFA_task */
}

/* USER CODE BEGIN Header_pc_task */
/**
* @brief Function implementing the pcTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_pc_task */
__weak void pc_task(void const * argument)
{
  /* USER CODE BEGIN pc_task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END pc_task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
