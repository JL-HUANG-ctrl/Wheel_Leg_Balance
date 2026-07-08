/*
 * @File         : bsp_rs485.c
 * @Author       : JL HUANG
 * @Date         : 2023-11-29 20:00:36
 * @LastEditTime : 2024-01-15 21:08:47
 * @FilePath     : \MDK-ARMc:\Users\JLH\Desktop\balance2\Bsp\bsp_rs485.c
 * @Description  : RS485使用USART1，即C板丝印的UART2(4Pin串口)
 */

#include "bsp_rs485.h"
#include "usart.h"
#include "app_GO1.h"

//extern DMA_HandleTypeDef  hdma_usart1_rx;
//extern DMA_HandleTypeDef  hdma_usart1_tx;

///**
// * @description  : 
// * @return        {*}
// */
//void RS485_TxEnable(void)
//{
//  __HAL_DMA_ENABLE(&hdma_usart1_tx);
//}

///**
// * @description  : 
// * @return        {*}
// */
//void RS485_TxDisable(void)
//{
//  __HAL_DMA_DISABLE(&hdma_usart1_tx);
//}

///**
// * @description  : 
// * @return        {*}
// */
//void RS485_RxEnable(void)
//{
//  __HAL_DMA_ENABLE(&hdma_usart1_rx);
//}

///**
// * @description  : 
// * @return        {*}
// */
//void RS485_RxDisable(void)
//{
//  __HAL_DMA_DISABLE(&hdma_usart1_rx);
//}

