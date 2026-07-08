/*
 * @File         : 
 * @Author       : JL HUANG
 * @Date         : 2024-04-21 00:25:52
 * @LastEditTime : 2024-05-14 18:40:47
 * @FilePath     : \MDK-ARMc:\Users\JLH\Desktop\newbalD\Components\Devices\app_link.c
 * @Description  : 
 */

#include "app_link.h"
#include "master_task.h"
#include "referee.h"
#include "refereeData_v1.6.h"


extern shoot_data_t shoot_data;
extern power_heat_data_t power_heat_data; 
extern robot_state_t robot_state; 
/**
 * @description  : 
 * @param         {int} x_int
 * @param         {float} x_min
 * @param         {float} x_max
 * @param         {int} bits
 * @return        {*}
 */
float Link_uint_to_float(int x_int,float x_min,float x_max,int bits)
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
int Link_float_to_uint(float x,float x_min,float x_max,int bits)
{
  float span = x_max - x_min;
  float offset = x_min;
  return (int) ((x-offset)*((float)((1<<bits)-1))/span);
}

/**
 * @description  : 
 * @param         {CAN_HandleTypeDef*} hcan
 * @return        {*}
 */
void Set_MasterLink_Data(CAN_HandleTypeDef* hcan)
{
  // uint16_t spd_x_tmp,spd_y_tmp,spd_w_tmp,leg_len_tmp;

  // spd_x_tmp = Link_float_to_uint(MasterLink.UP_Board_SetData.Spd_X,SPD_X_MAX,SPD_X_MIN,12);
  // spd_y_tmp = Link_float_to_uint(MasterLink.UP_Board_SetData.Spd_Y,SPD_Y_MAX,SPD_Y_MIN,12);
  // spd_w_tmp = Link_float_to_uint(MasterLink.UP_Board_SetData.Spd_W,SPD_W_MAX,SPD_W_MIN,12);
  // leg_len_tmp = Link_float_to_uint(MasterLink.UP_Board_SetData.LegLength,Leg_Length_MAX,Leg_Length_MIN,12);

  uint8_t Tx_Data[8] = {0};
  CAN_TxHeaderTypeDef CAN_TxHeader;
  
  CAN_TxHeader.IDE   = CAN_ID_STD;    //帧类型为标准帧
  CAN_TxHeader.RTR   = CAN_RTR_DATA;  //帧格式为数据帧
  CAN_TxHeader.DLC   = 0x08;          //数据长度为8
  CAN_TxHeader.StdId = DOWN_Board_Set_ID;      //数据ID标识符
  
  RefereeData.gunSpeed1=shoot_data.initial_speed;
  RefereeData.gunHeat1=power_heat_data.shooter_17mm_1_barrel_heat;
  RefereeData.maxHeat=robot_state.shooter_barrel_heat_limit;
   Tx_Data[0] = ( RefereeData.gunHeat1>> 8);
   Tx_Data[1] = (RefereeData.gunHeat1);
   Tx_Data[2] = ((uint16_t)RefereeData.gunSpeed1 >>8);
   Tx_Data[3] = ((uint16_t)RefereeData.gunSpeed1);
   Tx_Data[4] = (RefereeData.maxHeat>>8);
   Tx_Data[5] = RefereeData.maxHeat;
   Tx_Data[6] = (RefereeData.target);
   Tx_Data[7] = 0;
  
  HAL_CAN_AddTxMessage(hcan, &CAN_TxHeader, Tx_Data, 0);
  
  HAL_CAN_Start(hcan);
}

void Get_MasterLink_Info(Down_Board_Get_t* down_board_get_t,uint8_t data[8])
{
  down_board_get_t ->Spd_X_int       = (data[0] << 4) | (data[1]>>4);
  down_board_get_t ->Spd_Y_int       = ((data[1] & 0x0F) << 8) | data[2];
  down_board_get_t ->Spd_W_int       = (data[3] << 4) | (data[4]>>4);
  down_board_get_t ->LegLength_int   = ((data[4] & 0x0F) << 8) | data[5];
  down_board_get_t ->SysStartLog     = data[6] >> 7;
  down_board_get_t ->Mode_data       =(data[6]&0x40)>>6;

  down_board_get_t ->Spd_X       = Link_uint_to_float(down_board_get_t ->Spd_X_int,SPD_X_MAX,SPD_X_MIN,12);
  down_board_get_t ->Spd_Y       = Link_uint_to_float(down_board_get_t ->Spd_Y_int,SPD_Y_MAX,SPD_Y_MIN,12);
  down_board_get_t ->Spd_W       = Link_uint_to_float(down_board_get_t ->Spd_W_int,SPD_W_MAX,SPD_W_MIN,12);
  down_board_get_t ->LegLength   = Link_uint_to_float(down_board_get_t ->LegLength_int,Leg_Length_MAX,Leg_Length_MIN,12);
}
