/*
 * @File         : 
 * @Author       : JL HUANG
 * @Date         : 2023-11-21 11:51:11
 * @LastEditTime : 2024-04-17 00:39:35
 * @FilePath     : \MDK-ARMc:\Users\JLH\Desktop\newbalD\Components\Devices\app_RM_Motor.h
 * @Description  : 
 */

#ifndef __APP_RM_MOTOR_H
#define __APP_RM_MOTOR_H

#include "main.h"
#include "controller.h"

#define Wheel_Bus  hcan1

/**
 * @description  : 电机ID，用于收发报文判定
 * @return        {*}
 */
#define GM6020_SET_STD_ID						 0x1FF
#define GM6020_GET_STD_ID						 0x204

#define GM6020_ID_Yaw                0x01
#define GM6020_ID_Pitch              0x02

#define M3508_STD_ID			     			 0x200

#define M3508_ID_Left                0x01
#define M3508_ID_Right               0x02

#define M2006_STD_ID			     			 0x200

#define M2006_ID                     0x03

typedef struct 
{
  uint16_t encoder;      //转子机械角度(0~8191)
  int16_t  last_encoder;

  int16_t  speed;        //转子转速
  int16_t  current;      //转矩电流
  int8_t   temperature;  //温度(2006无此项)

  int32_t  round;         //转子圈数累计
  int32_t  total_encoder;
} RM_MotorMeasure_t;

typedef struct
{
  RM_MotorMeasure_t RM_MotorMeasure;
  PID_t             RM_MotorCtrl;
} RM_Motor_t;

void Set_RM_MotorCurrent(CAN_HandleTypeDef* hcan,uint32_t DeviceID,int16_t IQ1,int16_t IQ2,int16_t IQ3,int16_t IQ4);
void Get_RM_MotorInfo(RM_MotorMeasure_t* rm_motor_t, uint8_t data[8]);

#endif
