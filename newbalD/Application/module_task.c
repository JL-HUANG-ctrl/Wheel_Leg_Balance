/*
 * @File         : 
 * @Author       : JL HUANG
 * @Date         : 2023-11-26 16:10:36
 * @LastEditTime : 2023-11-26 16:43:37
 * @FilePath     : \MDK-ARMc:\Users\JLH\Desktop\balance\Application\module_task.c
 * @Description  : 
 */

#include "cmsis_os.h"

#include "module_task.h"
#include "tim.h"

void Module_Init(void)
{
  
}

void Module_Task(void)
{
  __HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_1,500);
  osDelay(2000);
  __HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_1,1000);
  osDelay(2000);
  __HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_1,1500);
  osDelay(2000);
  __HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_1,2000);
  osDelay(2000);
  __HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_1,2500);
  osDelay(2000);
}
