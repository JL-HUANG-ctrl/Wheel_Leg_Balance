/*
 * @File         : 
 * @Author       : JL HUANG
 * @Date         : 2024-04-14 16:49:42
 * @LastEditTime : 2024-04-21 13:23:23
 * @FilePath     : \MDK-ARMc:\Users\JLH\Desktop\newbalD\Components\Devices\app_DM8009.c
 * @Description  : 
 */

#include "app_DM8009.h"
#include "bsp_tim.h"

/**
 * @description  : 电机使能
 * @param         {CAN_HandleTypeDef*} hcan
 * @param         {uint32_t} id
 * @return        {*}
 */
void DM8009_Enable(CAN_HandleTypeDef* hcan,uint32_t id)
{
  uint8_t Tx_Data[8] = {0};
  CAN_TxHeaderTypeDef CAN_TxHeader;
  
  CAN_TxHeader.IDE   = CAN_ID_STD;    //帧类型为标准帧
  CAN_TxHeader.RTR   = CAN_RTR_DATA;  //帧格式为数据帧
  CAN_TxHeader.DLC   = 0x08;          //数据长度为8
  CAN_TxHeader.StdId = id;      //数据ID标识符
  
  Tx_Data[0] = 0xFF;
  Tx_Data[1] = 0xFF;
  Tx_Data[2] = 0xFF;
  Tx_Data[3] = 0xFF;
  Tx_Data[4] = 0xFF;
  Tx_Data[5] = 0xFF;
  Tx_Data[6] = 0xFF;
  Tx_Data[7] = 0xFC;
  
  HAL_CAN_AddTxMessage(hcan, &CAN_TxHeader, Tx_Data, 0);
  
  HAL_CAN_Start(hcan);
  
  Delay_us(250);
}

/**
 * @description  : 电机失能
 * @param         {CAN_HandleTypeDef*} hcan
 * @param         {uint32_t} id
 * @return        {*}
 */
void DM8009_Disable(CAN_HandleTypeDef* hcan,uint32_t id)
{
  uint8_t Tx_Data[8] = {0};
  CAN_TxHeaderTypeDef CAN_TxHeader;
  
  CAN_TxHeader.IDE   = CAN_ID_STD;    //帧类型为标准帧
  CAN_TxHeader.RTR   = CAN_RTR_DATA;  //帧格式为数据帧
  CAN_TxHeader.DLC   = 0x08;          //数据长度为8
  CAN_TxHeader.StdId = id;      //数据ID标识符
  
  Tx_Data[0] = 0xFF;
  Tx_Data[1] = 0xFF;
  Tx_Data[2] = 0xFF;
  Tx_Data[3] = 0xFF;
  Tx_Data[4] = 0xFF;
  Tx_Data[5] = 0xFF;
  Tx_Data[6] = 0xFF;
  Tx_Data[7] = 0xFD;
  
  HAL_CAN_AddTxMessage(hcan, &CAN_TxHeader, Tx_Data, 0);
  
  HAL_CAN_Start(hcan);

  Delay_us(250);
}

/**
 * @description  : 
 * @param         {int} x_int
 * @param         {float} x_min
 * @param         {float} x_max
 * @param         {int} bits
 * @return        {*}
 */
float DM8009_uint_to_float(int x_int,float x_min,float x_max,int bits)
{
  float span = x_max - x_min;
  float offset = x_min;
  return ((float)x_int)*span/((float)((1<<bits)-1)) + offset;
}

/**
 * @description  : 
 * @param         {float} x
 * @param         {float} x_min
 * @param         {float} x_max
 * @param         {int} bits
 * @return        {*}
 */
int DM8009_float_to_uint(float x,float x_min,float x_max,int bits)
{
  float span = x_max - x_min;
  float offset = x_min;
  return (int) ((x-offset)*((float)((1<<bits)-1))/span);
}

/**
 * @description  : 
 * @param         {CAN_HandleTypeDef*} hcan
 * @param         {uint32_t} id
 * @param         {float} pos
 * @param         {float} vel
 * @param         {float} torq
 * @param         {float} kp
 * @param         {float} kd
 * @return        {*}
 */
void Set_DM8009_MotorCurrent(CAN_HandleTypeDef* hcan,uint32_t id,float pos,float vel,float torq,float kp,float kd)
{
  uint16_t pos_tmp,vel_tmp,kp_tmp,kd_tmp,tor_tmp;
  
  pos_tmp = DM8009_float_to_uint(pos, P_MIN, P_MAX, 16);
  vel_tmp = DM8009_float_to_uint(vel, V_MIN, V_MAX, 12);
  kp_tmp  = DM8009_float_to_uint(kp, KP_MIN, KP_MAX, 12);
  kd_tmp  = DM8009_float_to_uint(kd, KD_MIN, KD_MAX, 12);
  tor_tmp = DM8009_float_to_uint(torq, T_MIN, T_MAX, 12);

  uint8_t Tx_Data[8] = {0};
  CAN_TxHeaderTypeDef CAN_TxHeader;
  
  CAN_TxHeader.IDE   = CAN_ID_STD;    //帧类型为标准帧
  CAN_TxHeader.RTR   = CAN_RTR_DATA;  //帧格式为数据帧
  CAN_TxHeader.DLC   = 0x08;          //数据长度为8
  CAN_TxHeader.StdId = id;      //数据ID标识符
  
  Tx_Data[0] = (pos_tmp >> 8);
  Tx_Data[1] = pos_tmp;
  Tx_Data[2] = (vel_tmp >> 4);
  Tx_Data[3] = ((vel_tmp & 0x0F) << 4)|(kp_tmp >> 8);
  Tx_Data[4] = kp_tmp;
  Tx_Data[5] = (kd_tmp >> 4);
  Tx_Data[6] = ((kd_tmp & 0x0F) << 4)|(tor_tmp >> 8);
  Tx_Data[7] = tor_tmp;
  
  HAL_CAN_AddTxMessage(hcan, &CAN_TxHeader, Tx_Data, 0);
  
  HAL_CAN_Start(hcan);

  /* 发完一个电机包后需要等待以匹配时序 */
  Delay_us(300);
}

/**
 * @description  : 
 * @param         {DM8009_MotorMeasure_t*} dm8009_motor_t
 * @param         {uint8_t} data
 * @return        {*}
 */
void Get_DM8009_MotorInfo(DM8009_MotorMeasure_t* dm8009_motor_t, uint8_t data[8])
{
  dm8009_motor_t->ID = data[0] & 0x0f;
  dm8009_motor_t->ERR = data[0] >> 4;

  dm8009_motor_t->Pos_int = (data[1] << 8) | data[2];
  dm8009_motor_t->Vel_int= (data[3] << 4) | (data[4]>>4);
  dm8009_motor_t->Tor_int = ((data[4] & 0x0F) << 8)|data[5];

  dm8009_motor_t->Position = DM8009_uint_to_float(dm8009_motor_t->Pos_int,P_MIN, P_MAX, 16);
  dm8009_motor_t->Velocity = DM8009_uint_to_float(dm8009_motor_t->Vel_int, V_MIN, V_MAX, 12);
  dm8009_motor_t->Torque = DM8009_uint_to_float(dm8009_motor_t->Tor_int, T_MIN, T_MAX, 12);

  dm8009_motor_t->T_MOS = data[6];
  dm8009_motor_t->T_Rotor = data[7];
}

/**
 * @description  : 设置电机相对零点
 * @param         {DM8009_Motor_t*} dm8009_motor_t
 * @return        {*}
 */
void DM8009_SetPosition(DM8009_Motor_t* dm8009_motor_t)
{
  dm8009_motor_t->offsetPosition = dm8009_motor_t->DM8009_MotorMeasure.Position;
}

// void DM8009_ResetPosition(CAN_HandleTypeDef* hcan,uint32_t tx_id,float pos,float vel,float torq,float kp,float kd)
// {

// }
