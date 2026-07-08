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

#include "ins_task.h"
#include "chassis_task.h"
#include "slip_defend.h"
#include "referee.h"
#include "master_task.h"
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
osThreadId INSTaskHandle;
osThreadId ChassisTaskHandle;
osThreadId ModuleTaskHandle;
osThreadId PIDTaskHandle;
osThreadId MasterLinkTaskHandle;
osThreadId Refree_TaskHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartINSTask(void const * argument);
void StartChassisTask(void const * argument);
void StartModuleTask(void const * argument);
void StartPIDTask(void const * argument);
void StartMasterLinkTask(void const * argument);
void StartRefree(void const * argument);

extern void MX_USB_DEVICE_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* GetTimerTaskMemory prototype (linked to static allocation support) */
void vApplicationGetTimerTaskMemory( StaticTask_t **ppxTimerTaskTCBBuffer, StackType_t **ppxTimerTaskStackBuffer, uint32_t *pulTimerTaskStackSize );

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

/* USER CODE BEGIN GET_TIMER_TASK_MEMORY */
static StaticTask_t xTimerTaskTCBBuffer;
static StackType_t xTimerStack[configTIMER_TASK_STACK_DEPTH];

void vApplicationGetTimerTaskMemory( StaticTask_t **ppxTimerTaskTCBBuffer, StackType_t **ppxTimerTaskStackBuffer, uint32_t *pulTimerTaskStackSize )
{
  *ppxTimerTaskTCBBuffer = &xTimerTaskTCBBuffer;
  *ppxTimerTaskStackBuffer = &xTimerStack[0];
  *pulTimerTaskStackSize = configTIMER_TASK_STACK_DEPTH;
  /* place for user code */
}
/* USER CODE END GET_TIMER_TASK_MEMORY */

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
  /* definition and creation of INSTask */
  osThreadDef(INSTask, StartINSTask, osPriorityRealtime, 0, 1024);
  INSTaskHandle = osThreadCreate(osThread(INSTask), NULL);

  /* definition and creation of ChassisTask */
  osThreadDef(ChassisTask, StartChassisTask, osPriorityNormal, 0, 256);
  ChassisTaskHandle = osThreadCreate(osThread(ChassisTask), NULL);

  /* definition and creation of ModuleTask */
  osThreadDef(ModuleTask, StartModuleTask, osPriorityNormal, 0, 256);
  ModuleTaskHandle = osThreadCreate(osThread(ModuleTask), NULL);

  /* definition and creation of PIDTask */
  osThreadDef(PIDTask, StartPIDTask, osPriorityLow, 0, 256);
  PIDTaskHandle = osThreadCreate(osThread(PIDTask), NULL);

  /* definition and creation of MasterLinkTask */
  osThreadDef(MasterLinkTask, StartMasterLinkTask, osPriorityAboveNormal, 0, 256);
  MasterLinkTaskHandle = osThreadCreate(osThread(MasterLinkTask), NULL);

  /* definition and creation of Refree_Task */
  osThreadDef(Refree_Task, StartRefree, osPriorityNormal, 0, 256);
  Refree_TaskHandle = osThreadCreate(osThread(Refree_Task), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartINSTask */
/**
  * @brief  Function implementing the INSTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartINSTask */
void StartINSTask(void const * argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN StartINSTask */
  INS_Init();
  /* Infinite loop */
  for(;;)
  {
    INS_Task();
    osDelay(1);
  }
  /* USER CODE END StartINSTask */
}

/* USER CODE BEGIN Header_StartChassisTask */
/**
* @brief Function implementing the ChassisTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartChassisTask */
void StartChassisTask(void const * argument)
{
  /* USER CODE BEGIN StartChassisTask */

  ChassisInit();
  /* 用于保存上次时间。调用后系统自动更新 */ 
  static portTickType PreviousWakeTime; 
  /* 设置延时时间，将时间转为节拍�? */ 
  const portTickType TimeIncrement = pdMS_TO_TICKS(2); 
  /* 获取当前系统时间 */ 
  PreviousWakeTime = osKernelSysTick();

  /* Infinite loop */
  for(;;)
  {
    ChassisTask();
    /* 调用绝对延时函数,任务时间间隔�?1个tick */
    osDelayUntil(&PreviousWakeTime,TimeIncrement);
    // osDelay(1);
  }
  /* USER CODE END StartChassisTask */
}

/* USER CODE BEGIN Header_StartModuleTask */
/**
* @brief Function implementing the ModuleTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartModuleTask */
void StartModuleTask(void const * argument)
{
  /* USER CODE BEGIN StartModuleTask */
  slip_defence_init();
  /* Infinite loop */
  for(;;)
  {
   slip_Defence_task();
    
    osDelay(1);
  }
  /* USER CODE END StartModuleTask */
}

/* USER CODE BEGIN Header_StartPIDTask */
/**
* @brief Function implementing the PIDTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartPIDTask */
void StartPIDTask(void const * argument)
{
  /* USER CODE BEGIN StartPIDTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartPIDTask */
}

/* USER CODE BEGIN Header_StartMasterLinkTask */
/**
* @brief Function implementing the MasterLinkTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartMasterLinkTask */
void StartMasterLinkTask(void const * argument)
{
  /* USER CODE BEGIN StartMasterLinkTask */
  /* Infinite loop */
  for(;;)
  {
    MasterLink_Task();
    osDelay(5);
  }
  /* USER CODE END StartMasterLinkTask */
}

/* USER CODE BEGIN Header_StartRefree */
/**
* @brief Function implementing the Refree_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartRefree */
void StartRefree(void const * argument)
{
  /* USER CODE BEGIN StartRefree */
    refereeINIT();
  /* Infinite loop */
  for(;;)
  {
    
    referee_unpack_fifo_data();
    RefereeDataUpdate(&RefereeData); 
    osDelay(5);
  }
  /* USER CODE END StartRefree */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
