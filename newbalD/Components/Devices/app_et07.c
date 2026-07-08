/*
 * @File         : 
 * @Author       : JL HUANG
 * @Date         : 2023-12-24 16:14:03
 * @LastEditTime : 2023-12-30 11:18:11
 * @FilePath     : \MDK-ARMc:\Users\JLH\Desktop\balance\Components\Devices\app_et07.c
 * @Description  : 
 */

#include "app_et07.h"


void Get_ET07_DataDecode(ET07_Data_t* et07_data_t,uint8_t* rx_buf)
{
  et07_data_t ->ET07_Start = rx_buf[0];

  et07_data_t ->RightTransverse_CH1 = ((int16_t)rx_buf[ 1] >> 0 | ((int16_t)rx_buf[ 2] << 8 )) & 0x07FF;
  et07_data_t ->RightDirection_CH2  = ((int16_t)rx_buf[ 2] >> 3 | ((int16_t)rx_buf[ 3] << 5 )) & 0x07FF;
  et07_data_t ->LeftDirection_CH3   = ((int16_t)rx_buf[ 3] >> 6 | ((int16_t)rx_buf[ 4] << 2 ) | (int16_t)rx_buf[5] << 10 ) & 0x07FF;
  et07_data_t ->LeftTransverse_CH4  = ((int16_t)rx_buf[ 5] >> 1 | ((int16_t)rx_buf[ 6] << 7 )) & 0x07FF;
  et07_data_t ->SA_CH5              = ((int16_t)rx_buf[ 6] >> 4 | ((int16_t)rx_buf[ 7] << 4 )) & 0x07FF;
  et07_data_t ->SB_CH6              = ((int16_t)rx_buf[ 7] >> 7 | ((int16_t)rx_buf[ 8] << 1 ) | (int16_t)rx_buf[9] << 9 ) & 0x07FF;
  et07_data_t ->SC_CH7              = ((int16_t)rx_buf[ 9] >> 2 | ((int16_t)rx_buf[10] << 6 )) & 0x07FF;
  et07_data_t ->SD_CH8              = ((int16_t)rx_buf[10] >> 5 | ((int16_t)rx_buf[11] << 3 )) & 0x07FF;
  et07_data_t ->V1_CH9              = ((int16_t)rx_buf[12] >> 0 | ((int16_t)rx_buf[13] << 8 )) & 0x07FF;
  et07_data_t ->V2_CH10             = ((int16_t)rx_buf[13] >> 3 | ((int16_t)rx_buf[14] << 5 )) & 0x07FF;

  et07_data_t ->ET07_Flag = rx_buf[23];
  et07_data_t ->ET07_End = rx_buf[24];
}

