/*
 * @File         : 
 * @Author       : JL HUANG
 * @Date         : 2023-11-20 20:45:33
 * @LastEditTime : 2024-05-09 14:36:43
 * @FilePath     : \MDK-ARMc:\Users\JLH\Desktop\newbalD\Bsp\bsp_can.c
 * @Description  : 
 */

#include "bsp_can.h"

#include "can.h"

#include "app_9025.h"
#include "app_DM8009.h"
#include "app_RM_Motor.h"
#include "app_link.h"

#include "chassis_task.h"
#include "master_task.h"

/**
 * @description  : CAN过滤器配置
 * @return       : 无
 */
void CAN_Filter_Init(void)
{
  CAN_FilterTypeDef CAN_FilterInitStructure;
  
  CAN_FilterInitStructure.FilterActivation     = CAN_FILTER_ENABLE;     //过滤器使能
  CAN_FilterInitStructure.FilterBank           = CAN1_FILTER_BANK;      //CAN1起始过滤器组编号
  CAN_FilterInitStructure.SlaveStartFilterBank = CAN2_FILTER_BANK;
  CAN_FilterInitStructure.FilterFIFOAssignment = CAN_RX_FIFO0;          //过滤器组关联FIFO0
  CAN_FilterInitStructure.FilterIdHigh         = 0x0000;
  CAN_FilterInitStructure.FilterIdLow          = 0x0000;
  CAN_FilterInitStructure.FilterMaskIdHigh     = 0x0000;
  CAN_FilterInitStructure.FilterMaskIdLow      = 0x0000;
  CAN_FilterInitStructure.FilterMode           = CAN_FILTERMODE_IDMASK; //掩码模式
  CAN_FilterInitStructure.FilterScale          = CAN_FILTERSCALE_32BIT; //32位
  
  HAL_CAN_ConfigFilter(&hcan1,&CAN_FilterInitStructure);
  HAL_CAN_Start(&hcan1);
  HAL_CAN_ActivateNotification(&hcan1,CAN_IT_RX_FIFO0_MSG_PENDING);     //开启FIFO0的接收中断
  
  CAN_FilterInitStructure.FilterFIFOAssignment = CAN_RX_FIFO1;
  CAN_FilterInitStructure.SlaveStartFilterBank = CAN2_FILTER_BANK;
  CAN_FilterInitStructure.FilterBank           = CAN2_FILTER_BANK;
  HAL_CAN_ConfigFilter(&hcan2,&CAN_FilterInitStructure);
  HAL_CAN_Start(&hcan2);
  HAL_CAN_ActivateNotification(&hcan2,CAN_IT_RX_FIFO1_MSG_PENDING);      //开启FIFO1的接收中断
}

/**
 * @description  : 接收FIFO0中断回调函数，处理CAN1接收到的报文
 * @param         {CAN_HandleTypeDef} *hcan
 * @return        {*}
 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  uint8_t Rx_Data[8] = {0};
  CAN_RxHeaderTypeDef CAN_RxHeader;

  HAL_CAN_GetRxMessage(hcan,CAN_RX_FIFO0,&CAN_RxHeader,Rx_Data);
  if(hcan -> Instance == CAN1)
  {
    switch(CAN_RxHeader.StdId)
    {
      case (Left_Front_Rx_ID):
        Get_DM8009_MotorInfo(&CHASSIS.DM8009_Motor[Left_Front_Joint].DM8009_MotorMeasure,Rx_Data);
        break;
      case (Left_Back_Rx_ID):
        Get_DM8009_MotorInfo(&CHASSIS.DM8009_Motor[Left_Back_Joint].DM8009_MotorMeasure,Rx_Data);
        break;
      case (Right_Front_Rx_ID):
        Get_DM8009_MotorInfo(&CHASSIS.DM8009_Motor[Right_Front_Joint].DM8009_MotorMeasure,Rx_Data);
        break;
      case (Right_Back_Rx_ID):
        Get_DM8009_MotorInfo(&CHASSIS.DM8009_Motor[Right_Back_Joint].DM8009_MotorMeasure,Rx_Data);
        break;
      default:
        break;
    }
  }
}

/**
 * @description  : 接收FIFO1中断回调函数，处理CAN2接收到的报文
 * @param         {CAN_HandleTypeDef} *hcan
 * @return        {*}
 */
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  uint8_t Rx_Data[8] = {0};
  CAN_RxHeaderTypeDef CAN_RxHeader;

  HAL_CAN_GetRxMessage(hcan,CAN_RX_FIFO1,&CAN_RxHeader,Rx_Data);
  if(hcan -> Instance == CAN2)
  {
    switch(CAN_RxHeader.StdId)
    {
      case (MF9025_STD_ID + MF9025_ID_Left):
        Get_MF9025_MotorInfo(&CHASSIS.MF9025_Motor[MF9025_ID_Left-1].MF9025_MotorMeasure,Rx_Data);
        break;
      case (MF9025_STD_ID + MF9025_ID_Right):
        Get_MF9025_MotorInfo(&CHASSIS.MF9025_Motor[MF9025_ID_Right-1].MF9025_MotorMeasure,Rx_Data);
        break;
      case (DOWN_Board_Get_ID):
        Get_MasterLink_Info(&MasterLink.Down_Board_GetData,Rx_Data);
        break;
      default:
        break;
    }
  }
}

/**
 * @description  : CAN错误中断回调函数，在这里重新打开接收中断
 * @param         {CAN_HandleTypeDef} *hcan
 * @return        {*}
 */
void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan)
{
  if (hcan->Instance == CAN1)
  {
    HAL_CAN_ActivateNotification(&hcan1,CAN_IT_RX_FIFO0_MSG_PENDING);
  }
  if (hcan->Instance == CAN2)
  {
    HAL_CAN_ActivateNotification(&hcan2,CAN_IT_RX_FIFO1_MSG_PENDING);
  }
}
