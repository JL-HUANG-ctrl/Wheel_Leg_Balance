/*
 * @File         : 
 * @Author       : JL HUANG
 * @Date         : 2024-04-14 17:02:41
 * @LastEditTime : 2024-04-21 12:53:38
 * @FilePath     : \MDK-ARMc:\Users\JLH\Desktop\newbalD\Sys\sys.h
 * @Description  : 
 */

#ifndef __SYS_H
#define __SYS_H

#include "main.h"

#include "spi.h"
#include "tim.h"
#include "can.h"

#include "BMI088driver.h"
#include "app_DM8009.h"
#include "app_buzzer.h"

#include "bsp_dwt.h"
#include "bsp_tim.h"
#include "bsp_can.h"

void SysInit(void);

#endif
