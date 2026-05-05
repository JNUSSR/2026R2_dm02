/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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

#include "4_Task/DJI_Motor_task.h"
#include "tsk_config_and_callback.h"
#include "ChassisTask.h"
#include "ClimbingTask2.h"
#include "DrawKFS_Task.h"
#include "ClampingTask.h"
#include "Uart_Task.h"

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
/* Definitions for MAVLinkTask */
osThreadId_t MAVLinkTaskHandle;
const osThreadAttr_t MAVLinkTask_attributes = {
  .name = "MAVLinkTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for ClimbingTask */
osThreadId_t ClimbingTaskHandle;
const osThreadAttr_t ClimbingTask_attributes = {
  .name = "ClimbingTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for DrawKFSTask */
osThreadId_t DrawKFSTaskHandle;
const osThreadAttr_t DrawKFSTask_attributes = {
  .name = "DrawKFSTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for ClampingTask */
osThreadId_t ClampingTaskHandle;
const osThreadAttr_t ClampingTask_attributes = {
  .name = "ClampingTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for ChassisTask */
osThreadId_t ChassisTaskHandle;
const osThreadAttr_t ChassisTask_attributes = {
  .name = "ChassisTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
osSemaphoreId_t uartSemaphoreHandle;
const osSemaphoreAttr_t uartSemaphore_attributes = {
  .name = "uartSemaphore"
};
/* USER CODE END FunctionPrototypes */

void StartMAVLinkTask(void *argument);
void StartTaskClimbing(void *argument);
void StartTaskDrawKFS(void *argument);
void StartTaskClamping(void *argument);
void StartTaskChassis(void *argument);

extern void MX_USB_DEVICE_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  Task_Init();

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
  /* creation of MAVLinkTask */
  MAVLinkTaskHandle = osThreadNew(StartMAVLinkTask, NULL, &MAVLinkTask_attributes);

  /* creation of ClimbingTask */
  ClimbingTaskHandle = osThreadNew(StartTaskClimbing, NULL, &ClimbingTask_attributes);

  /* creation of DrawKFSTask */
  DrawKFSTaskHandle = osThreadNew(StartTaskDrawKFS, NULL, &DrawKFSTask_attributes);

  /* creation of ClampingTask */
  ClampingTaskHandle = osThreadNew(StartTaskClamping, NULL, &ClampingTask_attributes);

  /* creation of ChassisTask */
  ChassisTaskHandle = osThreadNew(StartTaskChassis, NULL, &ChassisTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartMAVLinkTask */
/**
  * @brief  Function implementing the MAVLinkTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartMAVLinkTask */
void StartMAVLinkTask(void *argument)
{
  /* USER CODE BEGIN StartTaskMAVLink */
  /* Infinite loop */
  UartTask();
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartTaskMAVLink */
}

/* USER CODE BEGIN Header_StartTaskClimbing */
/**
* @brief Function implementing the ClimbingTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTaskClimbing */
void StartTaskClimbing(void *argument)
{
  /* USER CODE BEGIN StartTaskClimbing */
  /* Infinite loop */
  ClimbingTask();
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartTaskClimbing */
}

/* USER CODE BEGIN Header_StartTaskDrawKFS */
/**
* @brief Function implementing the DrawKFSTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTaskDrawKFS */
void StartTaskDrawKFS(void *argument)
{
  /* USER CODE BEGIN StartTaskDrawKFS */
  /* Infinite loop */
  DrawKFS_Task();
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartTaskDrawKFS */
}

/* USER CODE BEGIN Header_StartTaskClamping */
/**
* @brief Function implementing the ClampingTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTaskClamping */
void StartTaskClamping(void *argument)
{
  /* USER CODE BEGIN StartTaskClamping */
  /* Infinite loop */
  ClampingTask();
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartTaskClamping */
}

/* USER CODE BEGIN Header_StartTaskChassis */
/**
* @brief Function implementing the ChassisTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTaskChassis */
void StartTaskChassis(void *argument)
{
  /* USER CODE BEGIN StartTaskChassis */
  /* Infinite loop */
  Chassis_Task(argument);
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartTaskChassis */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

