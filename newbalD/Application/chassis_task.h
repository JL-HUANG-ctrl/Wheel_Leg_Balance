/*
 * @File         : 
 * @Author       : JL HUANG
 * @Date         : 2023-11-22 18:39:05
 * @LastEditTime : 2024-05-20 13:48:25
 * @FilePath     : \MDK-ARMc:\Users\JLH\Desktop\newbalD\Application\chassis_task.h
 * @Description  : 
 */

#ifndef __CHASSIS_TASK_H
#define __CHASSIS_TASK_H

#include "main.h"
#include "app_9025.h"
#include "app_RM_Motor.h"
#include "app_DM8009.h"
#include "vmc.h"
#include "lqr.h"
#include "controller.h"

/* 大腿收腿与水平夹角 */
#define LegParallelRadian 0.453786f

#define pi 3.141593f

/* 腿部上限位编码值 */
#define LeftFront_LimitPos   0.0f
#define LeftBack_LimitPos    0.0f
#define RightFront_LimitPos  0.0f
#define RightBack_LimitPos   0.0f

#define LegLeft  0
#define LegRight 1

/* 离地 */
#define OFF_GROUND 1
/* 未离地 */
#define ON_GROUND  0

/*小陀螺速度定义*/
#define Turn_speed 4000.0f

typedef struct
{
  Leg_VMC_t     Leg_VMC;
  PID_t         Support_F_Ctrl;
  LQR_Data_t    LQR_Data;

  /* 期望腿长 */
  float L0_Ref;
  /* 腿部虚拟力矩 */
  float TP0;

  /* 驱动轮支持力，用于离地检测 */
  float Fn;

  uint16_t LegStartCnt;
}Leg_Ctrl_t;

typedef struct
{
  /* 五连杆腿长 */
  float ThighL1;
  float ShankL2;
  float ShankL3;
  float ThighL4;
  float BodyL5;

  Leg_Ctrl_t LegCtrl[2];

  float x_average; //位移平均值

  float LegTim;
}Leg_t;

typedef struct
{
  PID_t TurnPD;
  float Yaw_offset;
}turn_t;

typedef struct 
{
  PID_t TunningPD;
  float delta_theta;
}leg_tunning_t;

typedef struct
{
  PID_t RollPD;
  float RollTarget;
}roll_t;

typedef struct
{
  MF9025_Motor_t MF9025_Motor[2];
  DM8009_Motor_t DM8009_Motor[4];
  
  Leg_t          Leg_Data;

  turn_t TurnCtrl;           //转向环
  leg_tunning_t TunningCtrl; //双腿协调
  roll_t RollCtrl;           //横滚角控制

  /* 底盘期望位移，外部控制接口 */
  float chassis_distance;
} chassis_t;

extern chassis_t CHASSIS;

void ChassisInit(void);
void ChassisTask(void);
void LegPosInit(void);
void ChassisReset(void);
void ChassisCtrl(void);

uint8_t Judge_StareLog(void);

void SetLQR_K_Feedback(Leg_Ctrl_t* leg_ctrl_t,float len);
void SetLQR_Left_State(void);
void SetLQR_Right_State(void);
void LQR_Ctrl(Leg_Ctrl_t* leg_ctrl_t);
void LegLengthCtrl(void);

void LegVMC_Position_Ctrl(Leg_Ctrl_t* leg_ctrl_t);
void LegVMC_F_Ctrl(Leg_Ctrl_t* leg_ctrl_t);

uint8_t GroundDetection(Leg_Ctrl_t* leg_ctrl_t);

void TurnInit(void);
void TurnAngleCtrl(void);
void LegSplitInit(void);
void LegSplitCtrl(void);
void LegRollInit(void);
void RollCtrl(void);

#endif
