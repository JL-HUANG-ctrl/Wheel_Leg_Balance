/*
 * @File         : 
 * @Author       : JL HUANG
 * @Date         : 2024-01-29 22:26:15
 * @LastEditTime : 2024-04-18 01:01:33
 * @FilePath     : \MDK-ARMc:\Users\JLH\Desktop\newbalD\Bsp\bsp_tim.h
 * @Description  : 
 */

#ifndef __BSP_TIM_H
#define __BSP_TIM_H

#include "main.h"

void RunTimStart(void);
float RunTimStop(void);

void Delay_us(uint16_t us);

void BuzzerStart(void);
void BuzzerStop(void);

void LED_Blue(void);
void LED_Green(void);
void LED_Red(void);
void LED_White(void);

#endif
