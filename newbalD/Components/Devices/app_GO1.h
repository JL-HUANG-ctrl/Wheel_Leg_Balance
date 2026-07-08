/*
 * @File         : app_GO1.h
 * @Author       : JL HUANG
 * @Date         : 2023-11-26 17:37:49
 * @LastEditTime : 2024-03-06 16:03:39
 * @FilePath     : \MDK-ARMc:\Users\JLH\Desktop\balance2\Components\Devices\app_GO1.h
 * @Description  : GO1电机驱动封装
 */

#ifndef __APP_GO1_H
#define __APP_GO1_H

#include "main.h"
#include "controller.h"

#include <stdint.h>

#define GO1_Reduction_Ratio 6.33f   //GO1电机减速比

#define pi                 3.141593f
#define pi2                6.2832f

/* 电机缓冲区大小，与电机收发数据大小对应 */
#define GO1_TX_buff_NUM    17
#define GO1_RX_buff_NUM    16

/* 电机模式 */
#define GO1_MODE_Default   0
#define GO1_MODE_FOC       1
#define GO1_MODE_Calibrate 2

/* 电机ID */
#define GO1_ID_LeftFront   1
#define GO1_ID_LeftBack    2
#define GO1_ID_RightFront  3
#define GO1_ID_RightBack   4

#define LegParallelRadian  0.356396f

#pragma pack(1)

typedef struct
{
  uint8_t ID    :4;          //电机ID:0~14  15表示向所有电机广播数据，此时无返回
  uint8_t Status:3;      //电机工作模式 0:锁定 1:FOC闭环 2:编码器校准
  uint8_t none  :1;        //高位保留
}GO1_MotorMode_t;

typedef struct
{
  float Torque;        //关节实际输出力矩
  float Speed;         //关节实际输出速度
  float Position;      //关节实际输出位置
  float FootForce;     //足端传感器数据
}GO1_MotMeaFloat_t;

typedef struct
{
  float Torque_Des;    //期望关节输出力矩
  float Speed_Des;     //期望关节速度
  float Position_Des;  //期望关节位置
  float K_Position;    //关节刚度系数
  float K_Speed;       //关节阻尼系数
}GO1_MotSetFloat_t;

typedef struct
{
  int16_t  T;          //实际关节输出转矩 N.m
  int16_t  W;          //实际关节输出速度 rad/s
  int32_t  Pos;        //实际关节输出位置 rad
  int8_t   Temp;       //电机温度 -128~127
  uint8_t  Err   :3;        //电机错误标识 0:正常 1:过热 2:过流 3:过压 4:编码器故障
  uint16_t Force :12;      //足端气压传感器数据 12bit (0~4095)
  uint8_t  none  :1;       //高位保留
}GO1_MotMeaInt_t;

typedef struct
{
  int16_t T_Des;       //期望关节输出扭矩 N.m
  int16_t W_Des;       //期望关节输出速度 rad/s
  int32_t Pos_Des;     //期望关节输出位置 rad
  int16_t K_Pos;       //期望关节刚度系数 -1.0~1.0
  int16_t K_W;         //期望关节阻尼系数 -1.0~1.0
}GO1_MotSetInt_t;

typedef struct
{
  /*** 接收电机原始数据 ***/
  uint8_t         GetHead[2];   //接收数据包头:0xFD 0xFE
  GO1_MotorMode_t GO1_MotorMode;
  GO1_MotMeaInt_t GO1_MotMeaInt;
  uint16_t        CRC_16;
}GO1_MeaCmd_t;

typedef struct
{
  /*** 发送电机原始数据 ***/
  uint8_t         SetHead[2];   //发送数据包头:0xFE 0xEE
  GO1_MotorMode_t GO1_MotorMode;
  GO1_MotSetInt_t GO1_MotSetInt;
  uint16_t        CRC_16;
}GO1_SetCmd_t;

/**
 * @description  : 接收电机状态参数结构体，共11Byte
 * @return        {*}
 */
typedef struct
{
  GO1_MeaCmd_t      GO1_MeaCmd;
  GO1_MotMeaFloat_t GO1_MotMeaFloat;
  float              OffSetPosition;  //E -20.42度-81.95     A 200.42-98.05    运动范围102.37  弧度=角度*pi/180  角度=180*弧度/pi
  float              OutPosition;
}GO1_MotorMeasure_t;

/**
 * @description  : 发送控制电机参数结构体，共12Byte
 * @return        {*}
 */
typedef struct
{
  GO1_SetCmd_t      GO1_SetCmd;
  GO1_MotSetFloat_t GO1_MotSetFloat;
}GO1_MotorSet_t;

typedef struct
{
  GO1_MotorSet_t     GO1_MotorSet;
  GO1_MotorMeasure_t GO1_MotorMeasure;
  PID_t              GO1_Ctrl;
}GO1_Motor_t;

void Set_GO1_MotorDecode(GO1_MotorSet_t* go1_motor_set_t,uint8_t id,float t_des,float w_des,float pos_des,float k_p,float k_w);
void Get_GO1_MotorDecode(GO1_MotorMeasure_t* go1_motor_get_t);
void Set_GO1_MotorInfo(GO1_MotorSet_t* go1_motor_set_t,uint8_t id,float t_des,float w_des,float pos_des,float k_p,float k_w);
void Get_GO1_MotorInfo(GO1_MotorMeasure_t* go1_motor_get_t);
void GO1_Update(GO1_MotorSet_t* go1_motor_set_t,GO1_MotorMeasure_t* go1_motor_get_t,
                  uint8_t id,float t_des,float w_des,float pos_des,float k_p,float k_w);

#pragma pack()

#endif
