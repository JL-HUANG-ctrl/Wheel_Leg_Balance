/*
 * @File         : 
 * @Author       : JL HUANG
 * @Date         : 2024-04-14 17:02:24
 * @LastEditTime : 2024-04-19 00:04:57
 * @FilePath     : \MDK-ARMc:\Users\JLH\Desktop\newbalD\Sys\sys.c
 * @Description  : 
 */

#include "sys.h"

/**
 * @description  : 系统初始化，在RTOS前完成
 * @return        {*}
 */
void SysInit(void)
{
  /* 板载LED初始化 */
  HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_3);

  /* 板载蜂鸣器初始化 */
  HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_3);

  /* PID时钟初始化 */
  DWT_Init(168);
  
  /* BMI088自检 */
  LED_Red();
  while (BMI088_init(&hspi1, 1) != BMI088_NO_ERROR)
      ;
  LED_Green();
  IMU_Start_Buzzer();

  /* CAN初始化 */
  CAN_Filter_Init();
  
  /* 关节电机使能 */
  DM8009_Enable(&JointBus,Left_Front_Tx_ID);
  DM8009_Enable(&JointBus,Left_Back_Tx_ID);
  DM8009_Enable(&JointBus,Right_Front_Tx_ID);
  DM8009_Enable(&JointBus,Right_Back_Tx_ID);
}
