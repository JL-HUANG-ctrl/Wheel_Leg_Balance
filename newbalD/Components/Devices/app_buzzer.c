/*
 * @File         : 
 * @Author       : JL HUANG
 * @Date         : 2024-04-18 01:37:12
 * @LastEditTime : 2024-04-18 01:38:22
 * @FilePath     : \MDK-ARMc:\Users\JLH\Desktop\newbalD\Components\Devices\app_buzzer.c
 * @Description  : 
 */

#include "app_buzzer.h"
#include "bsp_tim.h"

/**
 * @description  : 
 * @return        {*}
 */
void IMU_Start_Buzzer(void)
{
  BuzzerStart();
  HAL_Delay(100);
  BuzzerStop();
  HAL_Delay(100);
  BuzzerStart();
  HAL_Delay(100);
  BuzzerStop();
}
