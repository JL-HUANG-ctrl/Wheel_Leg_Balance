/*
 * @File         : 
 * @Author       : JL HUANG
 * @Date         : 2023-12-31 12:41:44
 * @LastEditTime : 2024-05-19 13:53:55
 * @FilePath     : \MDK-ARMc:\Users\JLH\Desktop\newbalD\Components\vmc.c
 * @Description  : 
 */

#include "vmc.h"
#include "arm_math.h"
#include "user_lib.h"
#include "chassis_task.h"

/**
 * @description  : VMC初始化
 * @return        {*}
 */
void LegVMC_Init(void)
{
  SetLegData();
  SetSupport_F_Data(200.0f,0.0f,150.0f,200.0f,0.0f,150.0f);
  //SetLegLen_Init();
}

/**
 * @description  : 设置腿部物理参数
 * @return        {*}
 */
void SetLegData(void)
{
  CHASSIS.Leg_Data.BodyL5 = 0.15f;
  CHASSIS.Leg_Data.ThighL1 = 0.15f;
  CHASSIS.Leg_Data.ThighL4 = 0.15f;
  CHASSIS.Leg_Data.ShankL2 = 0.27f;
  CHASSIS.Leg_Data.ShankL3 = 0.27f;
}

/**
 * @description  : 设置腿长控制器参数
 * @param         {float} kp0
 * @param         {float} kd0
 * @param         {float} kp1
 * @param         {float} kd1
 * @return        {*}
 */
void SetSupport_F_Data(float kp0,float ki0,float kd0,float kp1,float ki1,float kd1)
{
  PID_Init(&CHASSIS.Leg_Data.LegCtrl[0].Support_F_Ctrl,SUPPORT_MAX_OUT,SUPPORT_MAX_OUT,0,kp0,ki0,kd0,0,0,0,0,0,0);
  PID_Init(&CHASSIS.Leg_Data.LegCtrl[1].Support_F_Ctrl,SUPPORT_MAX_OUT,SUPPORT_MAX_OUT,0,kp1,ki1,kd1,0,0,0,0,0,0);
}

/**
 * @description  : 腿长初始化(0.104-0.405)
 * @return        {*}
 */
void SetLegLen_Init(void)
{
  CHASSIS.Leg_Data.LegCtrl[0].L0_Ref = 0.108f;
  CHASSIS.Leg_Data.LegCtrl[1].L0_Ref = 0.108f;
}

/**
 * @description  : 
 * @param         {Leg_Ctrl_t*} leg_ctrl_t
 * @return        {*}
 */
void LegVMC_Position_Ctrl(Leg_Ctrl_t* leg_ctrl_t)
{
  float X_B,X_D,Y_B,Y_D = 0.0f;
  //float L_BD = 0.0f;
  float A0,B0,C0 = 0.0f;

  X_B = CHASSIS.Leg_Data.ThighL1 * arm_cos_f32(leg_ctrl_t ->Leg_VMC.phi1);
  Y_B = CHASSIS.Leg_Data.ThighL1 * arm_sin_f32(leg_ctrl_t ->Leg_VMC.phi1);
  X_D = CHASSIS.Leg_Data.BodyL5 + CHASSIS.Leg_Data.ThighL4 * arm_cos_f32(leg_ctrl_t ->Leg_VMC.phi4);
  Y_D = CHASSIS.Leg_Data.ThighL4 * arm_sin_f32(leg_ctrl_t ->Leg_VMC.phi4);

  //arm_sqrt_f32(((X_D - X_B)*(X_D - X_B) + (Y_D - Y_B)*(Y_D - Y_B)),&L_BD);
  A0 = 2 * CHASSIS.Leg_Data.ShankL2 * (X_D - X_B);
  B0 = 2 * CHASSIS.Leg_Data.ShankL2 * (Y_D - Y_B);
  C0 = (X_D - X_B)*(X_D - X_B) + (Y_D - Y_B)*(Y_D - Y_B);

  leg_ctrl_t ->Leg_VMC.phi2 = 2 * atan2f(B0 + Sqrt(A0 * A0 + B0 * B0 - C0 * C0),A0 + C0);
  leg_ctrl_t ->Leg_VMC.phi3 = atan2f(Y_B - Y_D + CHASSIS.Leg_Data.ShankL2 * arm_sin_f32(leg_ctrl_t ->Leg_VMC.phi2),
                            X_B - X_D + CHASSIS.Leg_Data.ShankL2 * arm_cos_f32(leg_ctrl_t ->Leg_VMC.phi2));

  leg_ctrl_t ->Leg_VMC.X_C = X_B + CHASSIS.Leg_Data.ShankL2 * arm_cos_f32(leg_ctrl_t ->Leg_VMC.phi2);
  leg_ctrl_t ->Leg_VMC.Y_C = Y_B + CHASSIS.Leg_Data.ShankL2 * arm_sin_f32(leg_ctrl_t ->Leg_VMC.phi2);

  leg_ctrl_t ->Leg_VMC.phi0_last = leg_ctrl_t ->Leg_VMC.phi0;
  
  leg_ctrl_t ->Leg_VMC.L0  = Sqrt((leg_ctrl_t ->Leg_VMC.X_C - CHASSIS.Leg_Data.BodyL5/2) * (leg_ctrl_t ->Leg_VMC.X_C - CHASSIS.Leg_Data.BodyL5/2) + leg_ctrl_t ->Leg_VMC.Y_C * leg_ctrl_t ->Leg_VMC.Y_C);
  leg_ctrl_t->Leg_VMC.L0_dot=(leg_ctrl_t ->Leg_VMC.L0-leg_ctrl_t->Leg_VMC.L0_last)/0.001f;
  leg_ctrl_t ->Leg_VMC.phi0 = atan2f(leg_ctrl_t ->Leg_VMC.Y_C,leg_ctrl_t ->Leg_VMC.X_C - CHASSIS.Leg_Data.BodyL5/2);
  leg_ctrl_t ->Leg_VMC.phi0_dot = (leg_ctrl_t ->Leg_VMC.phi0 - leg_ctrl_t ->Leg_VMC.phi0_last) / 0.001f;
  leg_ctrl_t->Leg_VMC.L0_last=leg_ctrl_t->Leg_VMC.L0;
}

/**
 * @description  : 
 * @param         {Leg_Ctrl_t*} leg_ctrl_t
 * @return        {*}
 */
void LegVMC_F_Ctrl(Leg_Ctrl_t* leg_ctrl_t)
{
  float J11,J12,J21,J22 = 0.0f;

  J11 = (CHASSIS.Leg_Data.ThighL1 * arm_sin_f32(leg_ctrl_t ->Leg_VMC.phi0 - leg_ctrl_t ->Leg_VMC.phi3) * arm_sin_f32(leg_ctrl_t ->Leg_VMC.phi1 - leg_ctrl_t ->Leg_VMC.phi2))
        / arm_sin_f32(leg_ctrl_t ->Leg_VMC.phi3 - leg_ctrl_t ->Leg_VMC.phi2);
  J12 = (CHASSIS.Leg_Data.ThighL1 * arm_cos_f32(leg_ctrl_t ->Leg_VMC.phi0 - leg_ctrl_t ->Leg_VMC.phi3) * arm_sin_f32(leg_ctrl_t ->Leg_VMC.phi1 - leg_ctrl_t ->Leg_VMC.phi2))
        / (leg_ctrl_t ->Leg_VMC.L0 * arm_sin_f32(leg_ctrl_t ->Leg_VMC.phi3 - leg_ctrl_t ->Leg_VMC.phi2));
  
  J21 = (CHASSIS.Leg_Data.ThighL4 * arm_sin_f32(leg_ctrl_t ->Leg_VMC.phi0 - leg_ctrl_t ->Leg_VMC.phi2) * arm_sin_f32(leg_ctrl_t ->Leg_VMC.phi3 - leg_ctrl_t ->Leg_VMC.phi4))
        / arm_sin_f32(leg_ctrl_t ->Leg_VMC.phi3 - leg_ctrl_t ->Leg_VMC.phi2);
  J22 = (CHASSIS.Leg_Data.ThighL4 * arm_cos_f32(leg_ctrl_t ->Leg_VMC.phi0 - leg_ctrl_t ->Leg_VMC.phi2) * arm_sin_f32(leg_ctrl_t ->Leg_VMC.phi3 - leg_ctrl_t ->Leg_VMC.phi4))
        / (leg_ctrl_t ->Leg_VMC.L0 * arm_sin_f32(leg_ctrl_t ->Leg_VMC.phi3 - leg_ctrl_t ->Leg_VMC.phi2));
  
  leg_ctrl_t ->Leg_VMC.Tp1 = J11 * leg_ctrl_t ->Leg_VMC.F 
                                  + J12 * leg_ctrl_t ->TP0;
  leg_ctrl_t ->Leg_VMC.Tp2 = J21 * leg_ctrl_t ->Leg_VMC.F 
                                  + J22 * leg_ctrl_t ->TP0;
  
  // leg_ctrl_t ->Leg_VMC.Tp1= leg_ctrl_t ->Leg_VMC.Tp1/6.33f;//得到旋转轴期望力矩
  // leg_ctrl_t ->Leg_VMC.Tp2= leg_ctrl_t ->Leg_VMC.Tp2/6.33f;//得到旋转轴期望力矩
}

/**
 * @description  : 腿长控制器
 * @return        {*}
 */
void LegLengthCtrl(void)
{
  PID_Calculate(&CHASSIS.Leg_Data.LegCtrl[0].Support_F_Ctrl,CHASSIS.Leg_Data.LegCtrl[0].Leg_VMC.L0,CHASSIS.Leg_Data.LegCtrl[0].L0_Ref);
  PID_Calculate(&CHASSIS.Leg_Data.LegCtrl[1].Support_F_Ctrl,CHASSIS.Leg_Data.LegCtrl[1].Leg_VMC.L0,CHASSIS.Leg_Data.LegCtrl[1].L0_Ref);

  if(GroundDetection(&CHASSIS.Leg_Data.LegCtrl[LegLeft]) == OFF_GROUND)
  {
    CHASSIS.Leg_Data.LegCtrl[LegLeft].Leg_VMC.F = 80.0f / arm_cos_f32(CHASSIS.Leg_Data.LegCtrl[LegLeft].LQR_Data.LQR_State.theta) + CHASSIS.Leg_Data.LegCtrl[0].Support_F_Ctrl.Output;
  }
  else
  {
    CHASSIS.Leg_Data.LegCtrl[LegLeft].Leg_VMC.F = 80.0f / arm_cos_f32(CHASSIS.Leg_Data.LegCtrl[LegLeft].LQR_Data.LQR_State.theta)+ CHASSIS.Leg_Data.LegCtrl[0].Support_F_Ctrl.Output - CHASSIS.RollCtrl.RollPD.Output;
  }
  
  if(GroundDetection(&CHASSIS.Leg_Data.LegCtrl[LegRight]) == OFF_GROUND)
  {
    CHASSIS.Leg_Data.LegCtrl[LegRight].Leg_VMC.F = 80.0f /arm_cos_f32(CHASSIS.Leg_Data.LegCtrl[LegRight].LQR_Data.LQR_State.theta) + CHASSIS.Leg_Data.LegCtrl[1].Support_F_Ctrl.Output;
  }
  else
  {
    CHASSIS.Leg_Data.LegCtrl[LegRight].Leg_VMC.F = 80.0f /arm_cos_f32(CHASSIS.Leg_Data.LegCtrl[LegRight].LQR_Data.LQR_State.theta) + CHASSIS.Leg_Data.LegCtrl[1].Support_F_Ctrl.Output + CHASSIS.RollCtrl.RollPD.Output;
  }
}

/**
 * @description  : 
 * @param         {Leg_Ctrl_t*} leg_ctrl_t
 * @return        {*}
 */
uint8_t GroundDetection(Leg_Ctrl_t* leg_ctrl_t)
{
  leg_ctrl_t -> Leg_VMC.F0 = 80.0f / arm_cos_f32(leg_ctrl_t ->LQR_Data.LQR_State.theta) + leg_ctrl_t -> Support_F_Ctrl.Output;
  leg_ctrl_t -> Fn = leg_ctrl_t -> Leg_VMC.F0 * arm_cos_f32(leg_ctrl_t ->LQR_Data.LQR_State.theta) 
                      + leg_ctrl_t -> TP0 * arm_sin_f32(leg_ctrl_t ->LQR_Data.LQR_State.theta) / leg_ctrl_t -> Leg_VMC.L0
                      + WHEEL_G;
  if(leg_ctrl_t -> Fn < 60.0f)
  {
    return OFF_GROUND;
  }
  else
  {
    return ON_GROUND;
  }
}
