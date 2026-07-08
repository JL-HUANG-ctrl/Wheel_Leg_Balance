/*
 * @File         : 
 * @Author       : JL HUANG
 * @Date         : 2023-12-24 16:12:27
 * @LastEditTime : 2023-12-30 11:20:11
 * @FilePath     : \MDK-ARMc:\Users\JLH\Desktop\balance\Bsp\bsp_wbus.c
 * @Description  : 
 */

#include "bsp_wbus.h"
#include "usart.h"
#include "RC_task.h"

uint8_t W_BUSRxBuffer[2][RC_FRAME_NUM];

extern DMA_HandleTypeDef hdma_usart3_rx;

/**
 * @description  : 
 * @param         {uint8_t} *rx0_buf
 * @param         {uint8_t} *rx1_buf
 * @param         {uint16_t} dma_buf_num
 * @return        {*}
 */
void W_BUS_Init(uint8_t *rx0_buf, uint8_t *rx1_buf, uint16_t dma_buf_num)
{
  SET_BIT(huart3.Instance->CR3, USART_CR3_DMAR);  //使能DMA串口接收
  __HAL_UART_ENABLE_IT(&huart3,UART_IT_IDLE);     //使能空闲中断
  __HAL_DMA_DISABLE(&hdma_usart3_rx);             //关闭DMA
  while(hdma_usart3_rx.Instance->CR & DMA_SxCR_EN)
  {
    __HAL_DMA_DISABLE(&hdma_usart3_rx);
  }
  hdma_usart3_rx.Instance->PAR = (uint32_t) & (USART3->DR);
  
  hdma_usart3_rx.Instance->M0AR = (uint32_t)(rx0_buf); //内存缓冲区0
  hdma_usart3_rx.Instance->M1AR = (uint32_t)(rx1_buf); //内存缓冲区1
  hdma_usart3_rx.Instance->NDTR = dma_buf_num;         //数据长度
  
  SET_BIT(hdma_usart3_rx.Instance->CR, DMA_SxCR_DBM);  //使能双缓冲区
  
  __HAL_DMA_ENABLE(&hdma_usart3_rx);                   //使能DMA
}

/**
 * @description  : 
 * @return        {*}
 */
void W_BUS_IDLEHandler(void)
{
  if(huart3.Instance ->SR & UART_FLAG_RXNE)
  {
    __HAL_UART_CLEAR_FEFLAG(&huart3);
  }
  else if(USART3->SR & UART_FLAG_IDLE)
  {
    static uint16_t this_time_rx_len = 0;

    __HAL_UART_CLEAR_PEFLAG(&huart3);                     //清除中断标志位
    if((hdma_usart3_rx.Instance->CR & DMA_SxCR_CT) == RESET)
    {
      __HAL_DMA_DISABLE(&hdma_usart3_rx);                 //关闭DMA

      /*** 缓冲区0 ***/
      this_time_rx_len = W_BUS_RxBuff_NUM - hdma_usart3_rx.Instance->NDTR;//获取接收数据长度=设定长度-剩余长度
      hdma_usart3_rx.Instance->NDTR = W_BUS_RxBuff_NUM;    //重新设定数据长度

      /*** 缓冲区1 ***/
      hdma_usart3_rx.Instance->CR |= DMA_SxCR_CT;

      __HAL_DMA_ENABLE(&hdma_usart3_rx);                  //开启DMA

      if(this_time_rx_len == RC_FRAME_NUM)
      {
        /* 拆包函数 */
        Get_ET07_DataDecode(&RC_Data.ET07_DATA,W_BUSRxBuffer[0]);
      }
    }
    else
    {
      __HAL_DMA_DISABLE(&hdma_usart3_rx);                 //关闭DMA

      /*** 缓冲区1 ***/
      this_time_rx_len = W_BUS_RxBuff_NUM - hdma_usart3_rx.Instance->NDTR;//获取接收数据长度=设定长度-剩余长度
      hdma_usart3_rx.Instance->NDTR = W_BUS_RxBuff_NUM;    //重新设定数据长度

      /*** 缓冲区0 ***/
      DMA1_Stream1->CR &= ~(DMA_SxCR_CT);                 //USART3->RX挂载DMA1->Stream1

      __HAL_DMA_ENABLE(&hdma_usart3_rx);                  //开启DMA

      if(this_time_rx_len == RC_FRAME_NUM)
      {
        /* 拆包函数 */
        Get_ET07_DataDecode(&RC_Data.ET07_DATA,W_BUSRxBuffer[1]);
      }
    }
  }
}
