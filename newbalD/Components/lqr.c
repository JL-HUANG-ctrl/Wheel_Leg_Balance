/*
 * @File         : 
 * @Author       : JL HUANG
 * @Date         : 2024-01-05 17:19:33
 * @LastEditTime : 2024-05-20 13:54:21
 * @FilePath     : \MDK-ARMc:\Users\JLH\Desktop\newbalD\Components\lqr.c
 * @Description  : 
 */

#include "lqr.h"
#include "ins_task.h"
#include "app_9025.h"
#include "chassis_task.h"
#include "user_lib.h"
#include "master_task.h"
#include "slip_defend.h"
#include "chassis_power_control.h"

/**
 * @description  : 更新反馈K矩阵
 * @param         {Leg_Ctrl_t*} leg_ctrl_t
 * @param         {float} len
 * @return        {*}
 */
void SetLQR_K_Feedback(Leg_Ctrl_t* leg_ctrl_t,float len)
{
  leg_ctrl_t ->LQR_Data.K_Feedback[0][0] = K111 *len*len*len + K112 *len*len + K113 *len + K114;
  leg_ctrl_t ->LQR_Data.K_Feedback[0][1] = K121 *len*len*len + K122 *len*len + K123 *len + K124;
  leg_ctrl_t ->LQR_Data.K_Feedback[0][2] = K131 *len*len*len + K132 *len*len + K133 *len + K134;
  leg_ctrl_t ->LQR_Data.K_Feedback[0][3] = K141 *len*len*len + K142 *len*len + K143 *len + K144;
  leg_ctrl_t ->LQR_Data.K_Feedback[0][4] = K151 *len*len*len + K152 *len*len + K153 *len + K154;
  leg_ctrl_t ->LQR_Data.K_Feedback[0][5] = K161 *len*len*len + K162 *len*len + K163 *len + K164;

  leg_ctrl_t ->LQR_Data.K_Feedback[1][0] = K211 *len*len*len + K212 *len*len + K213 *len + K214;
  leg_ctrl_t ->LQR_Data.K_Feedback[1][1] = K221 *len*len*len + K222 *len*len + K223 *len + K224;
  leg_ctrl_t ->LQR_Data.K_Feedback[1][2] = K231 *len*len*len + K232 *len*len + K233 *len + K234;
  leg_ctrl_t ->LQR_Data.K_Feedback[1][3] = K241 *len*len*len + K242 *len*len + K243 *len + K244;
  leg_ctrl_t ->LQR_Data.K_Feedback[1][4] = K251 *len*len*len + K252 *len*len + K253 *len + K254;
  leg_ctrl_t ->LQR_Data.K_Feedback[1][5] = K261 *len*len*len + K262 *len*len + K263 *len + K264;
}

/**
 * @description  : 
 * @return        {*}
 */
void SetLQR_Left_State(void)
{
  /********** 左腿状态反馈 **********/
  CHASSIS.Leg_Data.LegCtrl[LegLeft].LQR_Data.LQR_State.theta_last = CHASSIS.Leg_Data.LegCtrl[LegLeft].LQR_Data.LQR_State.theta;
  CHASSIS.Leg_Data.LegCtrl[LegLeft].LQR_Data.LQR_State.phi_last = CHASSIS.Leg_Data.LegCtrl[LegLeft].LQR_Data.LQR_State.phi;
 // CHASSIS.Leg_Data.LegCtrl[0].LQR_Data.LQR_State.x_last = CHASSIS.Leg_Data.LegCtrl[0].LQR_Data.LQR_State.x;

  CHASSIS.Leg_Data.LegCtrl[LegLeft].LQR_Data.LQR_State.phi       = (-INS.Pitch + 2.0f) * 3.141593f / 180.0f;
  CHASSIS.Leg_Data.LegCtrl[LegLeft].LQR_Data.LQR_State.phi_dot   = -INS.Gyro[X] * 3.141593f / 180.0f;
  // CHASSIS.Leg_Data.LegCtrl[0].LQR_Data.LQR_State.phi_dot   = (CHASSIS.Leg_Data.LegCtrl[0].LQR_Data.LQR_State.phi - CHASSIS.Leg_Data.LegCtrl[0].LQR_Data.LQR_State.phi_last) / 0.002f;
  CHASSIS.Leg_Data.LegCtrl[LegLeft].LQR_Data.LQR_State.theta     = (3.141593f / 2.0f - CHASSIS.Leg_Data.LegCtrl[LegLeft].Leg_VMC.phi0 - CHASSIS.Leg_Data.LegCtrl[LegLeft].LQR_Data.LQR_State.phi);
  CHASSIS.Leg_Data.LegCtrl[LegLeft].LQR_Data.LQR_State.theta_dot = 
              (CHASSIS.Leg_Data.LegCtrl[LegLeft].LQR_Data.LQR_State.theta - CHASSIS.Leg_Data.LegCtrl[0].LQR_Data.LQR_State.theta_last) / (0.002f);
  
  CHASSIS.Leg_Data.LegCtrl[0].LQR_Data.LQR_State.x_dot = chassis_slip_defend.V_expect_l+INS.Gyro[2]*260.0f*0.001f;
  //CHASSIS.Leg_Data.LegCtrl[0].LQR_Data.LQR_State.theta_dot = - CHASSIS.Leg_Data.LegCtrl[0].Leg_VMC.phi0_dot - CHASSIS.Leg_Data.LegCtrl[0].LQR_Data.LQR_State.phi_dot;
  //CHASSIS.Leg_Data.LegCtrl[LegLeft].LQR_Data.LQR_State.x_dot     = CHASSIS.MF9025_Motor[MF9025_ID_Left-1].MF9025_MotorMeasure.speed_rad * WHEEL_R+INS.Gyro[2]*260.0f*0.001f;
  CHASSIS.Leg_Data.LegCtrl[LegLeft].LQR_Data.LQR_State.x         +=CHASSIS.Leg_Data.LegCtrl[LegLeft].LQR_Data.LQR_State.x_dot* (0.002f);
}

/**
 * @description  : 
 * @return        {*}
 */
void SetLQR_Right_State(void)
{
  /********** 右腿状态反馈 **********/
	CHASSIS.Leg_Data.LegCtrl[LegRight].LQR_Data.LQR_State.theta_last = CHASSIS.Leg_Data.LegCtrl[LegRight].LQR_Data.LQR_State.theta;
  CHASSIS.Leg_Data.LegCtrl[LegRight].LQR_Data.LQR_State.phi_last = CHASSIS.Leg_Data.LegCtrl[LegRight].LQR_Data.LQR_State.phi;
  
  CHASSIS.Leg_Data.LegCtrl[LegRight].LQR_Data.LQR_State.phi       = (-INS.Pitch + 2.0f) * 3.141593f / 180.0f;
  CHASSIS.Leg_Data.LegCtrl[LegRight].LQR_Data.LQR_State.phi_dot   = -INS.Gyro[X] * 3.141593f / 180.0f;
  // CHASSIS.Leg_Data.LegCtrl[1].LQR_Data.LQR_State.phi_dot   = (CHASSIS.Leg_Data.LegCtrl[1].LQR_Data.LQR_State.phi - CHASSIS.Leg_Data.LegCtrl[1].LQR_Data.LQR_State.phi_last) / 0.002f;
  CHASSIS.Leg_Data.LegCtrl[LegRight].LQR_Data.LQR_State.theta     = (3.141593f / 2.0f - CHASSIS.Leg_Data.LegCtrl[LegRight].Leg_VMC.phi0 - CHASSIS.Leg_Data.LegCtrl[LegRight].LQR_Data.LQR_State.phi);
  CHASSIS.Leg_Data.LegCtrl[LegRight].LQR_Data.LQR_State.theta_dot = 
              (CHASSIS.Leg_Data.LegCtrl[LegRight].LQR_Data.LQR_State.theta - CHASSIS.Leg_Data.LegCtrl[LegRight].LQR_Data.LQR_State.theta_last) / (0.002f);
  //CHASSIS.Leg_Data.LegCtrl[1].LQR_Data.LQR_State.theta_dot = - CHASSIS.Leg_Data.LegCtrl[1].Leg_VMC.phi0_dot - CHASSIS.Leg_Data.LegCtrl[1].LQR_Data.LQR_State.phi_dot;
  CHASSIS.Leg_Data.LegCtrl[1].LQR_Data.LQR_State.x_dot = chassis_slip_defend.V_expect_r-INS.Gyro[2]*260.0f*0.001f;
  //CHASSIS.Leg_Data.LegCtrl[LegRight].LQR_Data.LQR_State.x_dot     = 0.0f-CHASSIS.MF9025_Motor[MF9025_ID_Right-1].MF9025_MotorMeasure.speed_rad * WHEEL_R-INS.Gyro[2]*260.0f*0.001f;
	CHASSIS.Leg_Data.LegCtrl[LegRight].LQR_Data.LQR_State.x         += CHASSIS.Leg_Data.LegCtrl[LegRight].LQR_Data.LQR_State.x_dot*(0.002f);
}

/**
 * @description  : 
 * @return        {*}
 */
void Set_X_Average(void)
{
  CHASSIS.Leg_Data.x_average = (CHASSIS.Leg_Data.LegCtrl[LegLeft].LQR_Data.LQR_State.x + CHASSIS.Leg_Data.LegCtrl[LegRight].LQR_Data.LQR_State.x) / 2.0f;
  CHASSIS.Leg_Data.LegCtrl[LegLeft].LQR_Data.LQR_State.x = CHASSIS.Leg_Data.x_average;
  CHASSIS.Leg_Data.LegCtrl[LegRight].LQR_Data.LQR_State.x = CHASSIS.Leg_Data.x_average;
}

/**
 * @description  : LQR计算
 * @param         {Leg_Ctrl_t*} leg_ctrl_t
 * @return        {*}
 */

// float LQR_K11 =-13.9721f;
// float LQR_K12 = -1.6883f ;
// float LQR_K13 = -4.3046f;
// float LQR_K14 =  -3.9547f ;
// float LQR_K15 =   8.5249f  ;
// float LQR_K16 =     1.5402f;
// float LQR_K21 =10.5431f  ;
// float LQR_K22 =1.3254f ;
// float LQR_K23 = 5.0876f;
// float LQR_K24 =     4.2013f;
// float LQR_K25 =    20.5893f;
// float LQR_K26 =     2.0057f;


  // -21.8183   -2.2401   -4.6995   -5.0696   14.1760    2.2607
  //  19.3062    2.3326    7.2052    7.2301   33.4620    2.9289
float LQR_K11 = -21.8183f;
float LQR_K12 = -2.2401f;
float LQR_K13 = -4.6995f;
float LQR_K14 = -5.0696f;
float LQR_K15 =  14.1760f;
float LQR_K16 =  2.2607f;

float LQR_K21 =  19.3062f;
float LQR_K22 =  2.3326f;
float LQR_K23 =  7.2052f;
float LQR_K24 =  7.2301f;
float LQR_K25 =  33.4620f;
float LQR_K26 =  2.9289f;

void LQR_Ctrl(Leg_Ctrl_t* leg_ctrl_t)
{
  leg_ctrl_t ->LQR_Data.LQR_Ctrl.T_Wheel  = (leg_ctrl_t ->LQR_Data.K_Feedback[0][0] * (leg_ctrl_t ->LQR_Data.LQR_State.theta + 0.0f) 
                                            + leg_ctrl_t ->LQR_Data.K_Feedback[0][1] * leg_ctrl_t ->LQR_Data.LQR_State.theta_dot
                                            + leg_ctrl_t ->LQR_Data.K_Feedback[0][2] * (leg_ctrl_t ->LQR_Data.LQR_State.x)
                                          + leg_ctrl_t ->LQR_Data.K_Feedback[0][3] * (leg_ctrl_t ->LQR_Data.LQR_State.x_dot - CHASSIS.chassis_distance)
                                            + leg_ctrl_t ->LQR_Data.K_Feedback[0][4] * (leg_ctrl_t ->LQR_Data.LQR_State.phi + 0.0f)
                                            + leg_ctrl_t ->LQR_Data.K_Feedback[0][5] * leg_ctrl_t ->LQR_Data.LQR_State.phi_dot);
  
  leg_ctrl_t ->LQR_Data.LQR_Ctrl.Tp_Joint = (leg_ctrl_t ->LQR_Data.K_Feedback[1][0] * (leg_ctrl_t ->LQR_Data.LQR_State.theta + 0.0f)
                                            + leg_ctrl_t ->LQR_Data.K_Feedback[1][1] * leg_ctrl_t ->LQR_Data.LQR_State.theta_dot
                                            + leg_ctrl_t ->LQR_Data.K_Feedback[1][2] * (leg_ctrl_t ->LQR_Data.LQR_State.x)
                                            + leg_ctrl_t ->LQR_Data.K_Feedback[1][3] * (leg_ctrl_t ->LQR_Data.LQR_State.x_dot - CHASSIS.chassis_distance)
                                            + leg_ctrl_t ->LQR_Data.K_Feedback[1][4] * (leg_ctrl_t ->LQR_Data.LQR_State.phi + 0.0f)
                                            + leg_ctrl_t ->LQR_Data.K_Feedback[1][5] * leg_ctrl_t ->LQR_Data.LQR_State.phi_dot);
}

/**
 * @description  : LQR虚拟关节赋值，供VMC调用
 * @return        {*}
 */
void SetLQR_LeftVirtualJoint(void)
{
  if(GroundDetection(&CHASSIS.Leg_Data.LegCtrl[LegLeft]) == OFF_GROUND)
  {
    CHASSIS.Leg_Data.LegCtrl[LegLeft].TP0 = CHASSIS.Leg_Data.LegCtrl[LegLeft].LQR_Data.K_Feedback[1][0] * (CHASSIS.Leg_Data.LegCtrl[LegLeft].LQR_Data.LQR_State.theta + 0.0f)
                                            + CHASSIS.Leg_Data.LegCtrl[LegLeft].LQR_Data.K_Feedback[1][1] * CHASSIS.Leg_Data.LegCtrl[LegLeft].LQR_Data.LQR_State.theta_dot
                                            - CHASSIS.TunningCtrl.TunningPD.Output;
  }
  else
  {
    CHASSIS.Leg_Data.LegCtrl[LegLeft].TP0 = CHASSIS.Leg_Data.LegCtrl[LegLeft].LQR_Data.LQR_Ctrl.Tp_Joint - CHASSIS.TunningCtrl.TunningPD.Output;
  }
}

/**
 * @description  : LQR虚拟关节赋值，供VMC调用
 * @return        {*}
 */
void SetLQR_RightVirtualJoint(void)
{
  if(GroundDetection(&CHASSIS.Leg_Data.LegCtrl[LegRight]) == OFF_GROUND)
  {
    CHASSIS.Leg_Data.LegCtrl[LegRight].TP0 = CHASSIS.Leg_Data.LegCtrl[LegRight].LQR_Data.K_Feedback[1][0] * (CHASSIS.Leg_Data.LegCtrl[LegRight].LQR_Data.LQR_State.theta + 0.0f)
                                            + CHASSIS.Leg_Data.LegCtrl[LegRight].LQR_Data.K_Feedback[1][1]* CHASSIS.Leg_Data.LegCtrl[LegRight].LQR_Data.LQR_State.theta_dot
                                            + CHASSIS.TunningCtrl.TunningPD.Output;
  }
  else
  {
    CHASSIS.Leg_Data.LegCtrl[LegRight].TP0 = CHASSIS.Leg_Data.LegCtrl[LegRight].LQR_Data.LQR_Ctrl.Tp_Joint + CHASSIS.TunningCtrl.TunningPD.Output;
  }
}

int iq[2] = {0};
/**
 * @description  : LQR轮子力矩换算，输出轮子力矩
 * @return        {*}
 */
void SetLQR_Wheel(void)
{
  
//  iq[0] = (int)(65.0f * (CHASSIS.Leg_Data.LegCtrl[0].LQR_Data.LQR_Ctrl.T_Wheel / MF9025_TorqueConstant));
//  iq[1] = (int)(65.0f * (CHASSIS.Leg_Data.LegCtrl[1].LQR_Data.LQR_Ctrl.T_Wheel / MF9025_TorqueConstant));
  
  iq[0] = (int)(666.0f * CHASSIS.Leg_Data.LegCtrl[0].LQR_Data.LQR_Ctrl.T_Wheel);
  iq[1] = (int)(666.0f * CHASSIS.Leg_Data.LegCtrl[1].LQR_Data.LQR_Ctrl.T_Wheel);
  
  //iq_tor = (iq[0] + iq[1])/2;
  //Set_MF9025_MotorCurrent(&hcan1,-iq[0],-iq[1],0,0)
  iq[0]=0.0f-iq[0];
  iq[0] = iq[0] -CHASSIS.TurnCtrl.TurnPD.Output;
  iq[1] = iq[1] -CHASSIS.TurnCtrl.TurnPD.Output;
  
//  if(MasterLink.Down_Board_GetData.Mode_data==Control_Mode)
//  {
//    iq[0] = iq[0] -CHASSIS.TurnCtrl.TurnPD.Output;
//    iq[1] = iq[1] -CHASSIS.TurnCtrl.TurnPD.Output;
//  }

    iq[0] = iq[0] -CHASSIS.TurnCtrl.TurnPD.Output;
    iq[1] = iq[1] -CHASSIS.TurnCtrl.TurnPD.Output;
  
  iq[0] = Saturate_Int(iq[0],-2000,2000);
  iq[1] = Saturate_Int(iq[1],-2000,2000);
  
  if(GroundDetection(&CHASSIS.Leg_Data.LegCtrl[LegLeft]) == OFF_GROUND)
  {
    iq[0] = 0;
  }
  if(GroundDetection(&CHASSIS.Leg_Data.LegCtrl[LegRight]) == OFF_GROUND)
  {
    iq[1] = 0;
  }
  

  //chassis_power_control();
  Set_MF9025_MotorCurrent(&WheelBus,iq[0],iq[1],0,0);
  //Set_MF9025_MotorCurrent(&WheelBus,0,0,0,0);
}
