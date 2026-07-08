/*
 * @File         : 
 * @Author       : JL HUANG
 * @Date         : 2023-11-20 20:45:44
 * @LastEditTime : 2024-04-15 16:30:25
 * @FilePath     : \MDK-ARMc:\Users\JLH\Desktop\newbalD\Bsp\bsp_can.h
 * @Description  : 
 */

#ifndef __BSP_CAN_H
#define __BSP_CAN_H

#include "main.h"

#define  CAN1_FILTER_BANK  0    //CAN1起始过滤器组编号
#define  CAN2_FILTER_BANK  14   //CAN2起始过滤器组编号

void CAN_Filter_Init(void);

#endif
