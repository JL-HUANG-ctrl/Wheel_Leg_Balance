/*
 * @File         : app_9025.c
 * @Author       : JL HUANG
 * @Date         : 2023-11-20 20:44:46
 * @LastEditTime : 2024-04-19 16:48:16
 * @FilePath     : \MDK-ARMc:\Users\JLH\Desktop\newbalD\Components\Devices\app_9025.c
 * @Description  : 
 */

#include "app_9025.h"
#include "bsp_tim.h"

/**
 * @description  : 9025多电机电流发送函数
 * @param         {CAN_HandleTypeDef*} hcan
 * @param         {int16_t} IQ1             :范围-2000 2000，对应实际转矩电流-32A 32A
 * @param         {int16_t} IQ2
 * @param         {int16_t} IQ3
 * @param         {int16_t} IQ4
 * @return        {*}
 */
void Set_MF9025_MotorCurrent(CAN_HandleTypeDef* hcan,int16_t IQ1,int16_t IQ2,int16_t IQ3,int16_t IQ4)
{
	uint8_t Tx_Data[8] = {0};
	CAN_TxHeaderTypeDef CAN_TxHeader;
	
	CAN_TxHeader.IDE   = CAN_ID_STD;                //帧类型为标准帧
  CAN_TxHeader.RTR   = CAN_RTR_DATA;              //帧格式为数据帧
  CAN_TxHeader.DLC   = 0x08;                      //数据长度为8
  CAN_TxHeader.StdId = MF9025_STD_BOARDCAST_ID;   //数据ID标识符
	
	Tx_Data[0] = IQ1;
	Tx_Data[1] = (IQ1 >> 8);
	Tx_Data[2] = IQ2;
	Tx_Data[3] = (IQ2 >> 8);
	Tx_Data[4] = IQ3;
	Tx_Data[5] = (IQ3 >> 8);
	Tx_Data[6] = IQ4;
	Tx_Data[7] = (IQ4 >> 8);
	
	HAL_CAN_AddTxMessage(hcan, &CAN_TxHeader, Tx_Data, 0);
	
  HAL_CAN_Start(hcan);
  Delay_us(600);
}

/**
 * @description  : MF9025电机拆包函数，每向电机发送一次指令，电机按ID依次返回报文
 * @param         {MF9025_Motor_t*} mf9025_motor_t
 * @param         {uint8_t} data
 * @return        {*}
 */
void Get_MF9025_MotorInfo(MF9025_MotorMeasure_t* mf9025_motor_t, uint8_t data[8])
{
		mf9025_motor_t -> last_encoder = mf9025_motor_t -> encoder;             //保留上次转子角度
		
		mf9025_motor_t -> temperature  = (int8_t  )(data[1]);                //转子温度
		mf9025_motor_t -> current      = (int16_t )(data[2] | data[3] << 8); //转矩电流(real_current=current*5.f/65535.f)
		mf9025_motor_t -> speed        = (int16_t )(data[4] | data[5] << 8); //转子转速(每秒转过度数)
		mf9025_motor_t -> encoder      = (uint16_t)(data[6] | data[7] << 8); //本次转子机械角度
		mf9025_motor_t -> speed_rad    = mf9025_motor_t -> speed * 3.141593f / 180.0f;
//		if((mf9025_motor_t -> encoder) - (mf9025_motor_t -> last_encoder) > (65535/2))
//		{
//			mf9025_motor_t -> round --;
//		}
//		if((mf9025_motor_t -> encoder) - (mf9025_motor_t -> last_encoder) < -(65535/2))
//		{
//			mf9025_motor_t -> round ++;
//		}
//		mf9025_motor_t ->total_encoder = mf9025_motor_t -> round * 65535 + mf9025_motor_t -> encoder; //总编码值，需要减去初始编码值
}
