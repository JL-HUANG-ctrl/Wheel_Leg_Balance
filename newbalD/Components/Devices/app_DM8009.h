/*
 * @File         : 
 * @Author       : JL HUANG
 * @Date         : 2024-04-14 16:50:01
 * @LastEditTime : 2024-04-21 13:15:03
 * @FilePath     : \MDK-ARMc:\Users\JLH\Desktop\newbalD\Components\Devices\app_DM8009.h
 * @Description  : 
 */

#ifndef __APP_DM8009_H
#define __APP_DM8009_H

#include "main.h"
#include "can.h"

/* 总线选择 */
#define JointBus          hcan1

/* 电机数组标识符 */
#define Left_Front_Joint  0
#define Left_Back_Joint   1
#define Right_Front_Joint 2
#define Right_Back_Joint  3

/* 电机ID标识符 */
#define Left_Front_Tx_ID  0x00
#define Left_Front_Rx_ID  0x01

#define Left_Back_Tx_ID   0x02
#define Left_Back_Rx_ID   0x03

#define Right_Front_Tx_ID 0x04
#define Right_Front_Rx_ID 0x05

#define Right_Back_Tx_ID  0x06
#define Right_Back_Rx_ID  0x07

/* 电机参数限幅 */
#define P_MIN -12.5f
#define P_MAX 12.5f

#define V_MIN -45.0f
#define V_MAX 45.0f

#define T_MIN -54.0f
#define T_MAX 54.0f

#define KP_MIN 0.0f
#define KP_MAX 500.0f

#define KD_MIN 0.0f
#define KD_MAX 5.0f

/**
 * @description  : 电机错误码
 * @return        {*}
 */
typedef enum
{
  ERR_Disable              = 0x00,  //失能
  ERR_Enable               = 0x01,  //使能
  ERR_OverVoltage          = 0x08,  //过压
  ERR_UnderVoltage         = 0x09,  //欠压
  ERR_OverCurrent          = 0x0A,  //过流
  ERR_MOS_OverTemperture   = 0x0B,  //MOS过温
  ERR_Rotor_OverTemperture = 0x0C,  //线圈过温
  ERR_Loss_Communicate     = 0x0D,  //通讯丢失
  ERR_OverLoad             = 0x0E   //过载
}DM8009_ERR_State;

/**
 * @description  : DM8009电机接收报文结构体
 * @return        {*}
 */
typedef struct
{
  uint8_t ID;
  uint8_t ERR;

  int Pos_int;
  int Vel_int;
  int Tor_int;

  float Position;
  float Velocity;
  float Torque;

  uint8_t T_MOS;
  uint8_t T_Rotor;
}DM8009_MotorMeasure_t;

typedef struct
{
  DM8009_MotorMeasure_t DM8009_MotorMeasure;
  float offsetPosition;
  // PID_t                 MF9025_Ctrl;
} DM8009_Motor_t;

void DM8009_Enable(CAN_HandleTypeDef* hcan,uint32_t id);
void DM8009_Disable(CAN_HandleTypeDef* hcan,uint32_t id);

float DM8009_uint_to_float(int x_int,float x_min,float x_max,int bits);
int DM8009_float_to_uint(float x,float x_min,float x_max,int bits);

void Set_DM8009_MotorCurrent(CAN_HandleTypeDef* hcan,uint32_t id,float pos,float vel,float torq,float kp,float kd);
void Get_DM8009_MotorInfo(DM8009_MotorMeasure_t* dm8009_motor_t, uint8_t data[8]);

void DM8009_SetPosition(DM8009_Motor_t* dm8009_motor_t);

#endif
