/*
 * @File         : 
 * @Author       : JL HUANG
 * @Date         : 2023-11-20 20:45:12
 * @LastEditTime : 2024-05-07 01:40:13
 * @FilePath     : \MDK-ARMc:\Users\JLH\Desktop\newbalD\Components\Devices\app_9025.h
 * @Description  : 
 */

#ifndef __APP_9025_H
#define __APP_9025_H

#include "main.h"
#include "controller.h"

/* 总线选择 */
#define WheelBus          hcan2

/**
 * @description  : 电机ID，用于收发报文判定
 * @return        {*}
 */
#define MF9025_STD_ID						 0x140   //9025单电机控制标识符
#define MF9025_STD_BOARDCAST_ID	 0x280   //9025多电机控制标识符

#define MF9025_ID_Left           0x01
#define MF9025_ID_Right          0x02

/* 转矩电流最大16.5  转矩常数0.32  峰值扭矩4.5 */
/* 扭矩 / 0.32 / 最大电流 = iq / 2000 */
#define MF9025_TorqueConstant    0.32f

/**
 * @description  : 9025电机接收报文结构体
 * @return        {*}
 */
typedef struct 
{
  uint8_t  cmd;         //命令字节

  int8_t   temperature; //温度
  int16_t  current;     //转矩电流(-2048 2048) (-16.5A 16.5A)
  int16_t  speed;       //电机转速(-32768 32767)
  uint16_t encoder;     //编码值(0~65535)
  int16_t  last_encoder;
  float    speed_rad;
  
  int32_t  round;
  int32_t  total_encoder;
} MF9025_MotorMeasure_t;

typedef struct
{
  MF9025_MotorMeasure_t MF9025_MotorMeasure;
  // PID_t                 MF9025_Ctrl;
} MF9025_Motor_t;


void Set_MF9025_MotorCurrent(CAN_HandleTypeDef* hcan,int16_t IQ1,int16_t IQ2,int16_t IQ3,int16_t IQ4);
void Get_MF9025_MotorInfo(MF9025_MotorMeasure_t* mf9025_motor_t, uint8_t data[8]);

#endif
