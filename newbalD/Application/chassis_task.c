/*
 * @File         : 
 * @Author       : JL HUANG
 * @Date         : 2023-11-22 18:38:50
 * @LastEditTime : 2024-05-19 12:08:34
 * @FilePath     : \MDK-ARMc:\Users\JLH\Desktop\newbalD\Application\chassis_task.c
 * @Description  : R标右边为Pitch正方向，R标上边为Roll轴正方向
 */

#include "chassis_task.h"
#include "master_task.h"
#include "ins_task.h"

#include "bsp_tim.h"

chassis_t CHASSIS;

/**
 * @description  : 腿部姿态初始化
 * @return        {*}
 */
void ChassisInit(void)
{
  /* 腿部物理参数，腿长控制初始化 */
  LegVMC_Init();
}

/**
 * @description  : 底盘任务函数入口
 * @return        {*}
 */
void ChassisTask(void)
{
  //ChassisReset();
//  Set_MF9025_MotorCurrent(&WheelBus,100,-100,0,0);
  
  vTaskSuspendAll(); //开启设备锁，防止任务打断
  /* 由遥控使能摇杆判断底盘状态 */
  switch(Judge_StareLog())
  {
    case(STOP_SYS):
      ChassisReset();
      break;
    case(START_SYS):
      /* 关节电机使能 */
      DM8009_Enable(&JointBus,Left_Front_Tx_ID);
      DM8009_Enable(&JointBus,Left_Back_Tx_ID);
      DM8009_Enable(&JointBus,Right_Front_Tx_ID);
      DM8009_Enable(&JointBus,Right_Back_Tx_ID);
    
      LegPosInit();
      CHASSIS.Leg_Data.LegCtrl[LegLeft].LQR_Data.LQR_State.x = 0;
      CHASSIS.Leg_Data.LegCtrl[LegRight].LQR_Data.LQR_State.x = 0;
      /* 转向环初始化 */
      TurnInit();
      /* 劈叉环初始化 */
      LegSplitInit();
      /* 横滚角补偿初始化 */
      LegRollInit();
      break;
    case(START_END):
      ChassisCtrl();
      break;
    case(STOP_END):
      ChassisReset();
      break;
    default:
      break;
  }
  xTaskResumeAll(); //关闭设备锁
}

/**
 * @description  : 
 * @return        {*}
 */
uint8_t Judge_StareLog(void)
{
  uint8_t Log = STOP_Log;
  MasterLink.Down_Board_GetData.SysLog[1] = MasterLink.Down_Board_GetData.SysLog[0];
  MasterLink.Down_Board_GetData.SysLog[0] = MasterLink.Down_Board_GetData.SysStartLog;
  if(MasterLink.Down_Board_GetData.SysLog[0] == MasterLink.Down_Board_GetData.SysLog[1])
  {
    if(MasterLink.Down_Board_GetData.SysLog[0] == STOP_Log)
    {
      Log = STOP_END;
    }
    else if(MasterLink.Down_Board_GetData.SysLog[0] == START_Log)
    {
      Log = START_END;
    }
  }
  else if(MasterLink.Down_Board_GetData.SysLog[0] != MasterLink.Down_Board_GetData.SysLog[1])
  {
    if(MasterLink.Down_Board_GetData.SysLog[0] == STOP_Log)
    {
      Log = STOP_SYS;
    }
    else if(MasterLink.Down_Board_GetData.SysLog[0] == START_Log)
    {
      Log = START_SYS;
    }
  }
  return Log;
}

/**
 * @description  : 腿姿态初始化
 * @return        {*}
 */
void LegPosInit(void)
{
  /* 收腿至关节上限位 */
  Set_DM8009_MotorCurrent(&JointBus,Left_Front_Tx_ID,LeftFront_LimitPos,0,0.0,6,1.2);
  Set_DM8009_MotorCurrent(&JointBus,Left_Back_Tx_ID,LeftBack_LimitPos,0,0.0,6,1.2);
  Set_DM8009_MotorCurrent(&JointBus,Right_Front_Tx_ID,RightFront_LimitPos,0,0.0,6,1.2);
  Set_DM8009_MotorCurrent(&JointBus,Right_Back_Tx_ID,RightBack_LimitPos,0,0.0,6,1.2);
  
  HAL_Delay(1000);
}

/**
 * @description  : 底盘复位
 * @return        {*}
 */
void ChassisReset(void)
{
  Set_MF9025_MotorCurrent(&WheelBus,0,0,0,0);
  Set_DM8009_MotorCurrent(&JointBus,Left_Front_Tx_ID,0,0,0,0,0);
  Set_DM8009_MotorCurrent(&JointBus,Left_Back_Tx_ID,0,0,0,0,0);
  Set_DM8009_MotorCurrent(&JointBus,Right_Front_Tx_ID,0,0,0,0,0);
  Set_DM8009_MotorCurrent(&JointBus,Right_Back_Tx_ID,0,0,0,0,0);
}

/**
 * @description  : 转向环初始化
 * @return        {*}
 */
void TurnInit(void)
{
  CHASSIS.TurnCtrl.Yaw_offset = 0.0f;
//  PID_Init(&CHASSIS.TurnCtrl.TurnPD,2000,2000,0,20.0f,0.0,30.0f,0,0,0,0,0,0);
  PID_Init(&CHASSIS.TurnCtrl.TurnPD,2000,2000,0,1500.0f,0.0f,0.03f,0,0,0,0,0,0);
  CHASSIS.TurnCtrl.Yaw_offset = INS.Yaw + 180.0f;
}

/**
 * @description  : 双腿协调 2.1.2
 * @return        {*}
 */
void LegSplitInit(void)
{
  PID_Init(&CHASSIS.TunningCtrl.TunningPD,100,100,0,30.0f,0.0f,0.0f,0,0,0,0,0,0);
}

/**
 * @description  : 横滚角初始化
 * @return        {*}
 */
void LegRollInit(void)
{
  PID_Init(&CHASSIS.RollCtrl.RollPD,200,200,0,8.0f,0.0,0.0f,0,0,0,0,0,0);
  CHASSIS.RollCtrl.RollTarget = 0.0f;
}

/**
 * @description  : 底盘控制
 * @return        {*}
 */
void ChassisCtrl(void)
{
  RunTimStart();
  /* Todo:模式判断，是否侧面对敌 */
  CHASSIS.chassis_distance = MasterLink.Down_Board_GetData.Spd_Y;
  if(fabs(CHASSIS.chassis_distance) >= 0.05)
  {
    CHASSIS.Leg_Data.LegCtrl[LegLeft].LQR_Data.LQR_State.x = 0;
    CHASSIS.Leg_Data.LegCtrl[LegRight].LQR_Data.LQR_State.x = 0;
  }
  else
  {
    CHASSIS.chassis_distance = 0;
  }
  
  
  //CHASSIS.chassis_distance = 0;
  
  /* VMC角度坐标转换 */
  CHASSIS.Leg_Data.LegCtrl[0].Leg_VMC.phi1 = pi - ( CHASSIS.DM8009_Motor[Left_Front_Joint].DM8009_MotorMeasure.Position - LegParallelRadian);
  CHASSIS.Leg_Data.LegCtrl[0].Leg_VMC.phi4 = (-CHASSIS.DM8009_Motor[Left_Back_Joint].DM8009_MotorMeasure.Position) - LegParallelRadian;
  CHASSIS.Leg_Data.LegCtrl[1].Leg_VMC.phi1 = pi - ( (-CHASSIS.DM8009_Motor[Right_Front_Joint].DM8009_MotorMeasure.Position) - LegParallelRadian);
  CHASSIS.Leg_Data.LegCtrl[1].Leg_VMC.phi4 = CHASSIS.DM8009_Motor[Right_Back_Joint].DM8009_MotorMeasure.Position - LegParallelRadian;
  
  /* 期望腿长获取 */
  float ref_length;
  ref_length=(MasterLink.Down_Board_GetData.LegLength)/500.0f;
  
  if(ref_length<0.000005f&&ref_length>-0.000005f)
    ref_length=0.0f;
  
  CHASSIS.Leg_Data.LegCtrl[LegLeft].L0_Ref += ref_length;
  CHASSIS.Leg_Data.LegCtrl[LegRight].L0_Ref += ref_length;
  
  /*腿部限位*/
  if( CHASSIS.Leg_Data.LegCtrl[LegLeft].L0_Ref<0.104f)
    CHASSIS.Leg_Data.LegCtrl[LegLeft].L0_Ref=0.104f;
  if(CHASSIS.Leg_Data.LegCtrl[LegRight].L0_Ref<0.104f)
    CHASSIS.Leg_Data.LegCtrl[LegRight].L0_Ref=0.104f;
  
  if(CHASSIS.Leg_Data.LegCtrl[LegLeft].L0_Ref>0.405f)
  CHASSIS.Leg_Data.LegCtrl[LegLeft].L0_Ref=0.405f;
  if(CHASSIS.Leg_Data.LegCtrl[LegRight].L0_Ref>0.405f)
    CHASSIS.Leg_Data.LegCtrl[LegRight].L0_Ref=0.405f;
  
  
  

  // CHASSIS.Leg_Data.LegCtrl[LegLeft].L0_Ref = 0.2f;
  // CHASSIS.Leg_Data.LegCtrl[LegRight].L0_Ref = 0.2f;
  
  /* 横滚角补偿 */
  RollCtrl();
  /* 腿长控制器计算 */
  LegLengthCtrl();
  /* 双腿协调计算 */
  LegSplitCtrl();

  Set_X_Average();

  /******************** 左腿计算 ********************/
  /* 腿长拟合反馈矩阵 */
  SetLQR_K_Feedback(&CHASSIS.Leg_Data.LegCtrl[LegLeft],CHASSIS.Leg_Data.LegCtrl[LegLeft].Leg_VMC.L0);
  /* VMC运动学解算 */
  LegVMC_Position_Ctrl(&CHASSIS.Leg_Data.LegCtrl[LegLeft]);
  /* 左腿状态反馈 */
  SetLQR_Left_State();
  /* LQR计算 */
  LQR_Ctrl(&CHASSIS.Leg_Data.LegCtrl[LegLeft]);
  /* LQR虚拟关节赋值 */
  SetLQR_LeftVirtualJoint();
  /* VMC动力学解算 */
  LegVMC_F_Ctrl(&CHASSIS.Leg_Data.LegCtrl[LegLeft]);
  /* 关节输出 */
  if(CHASSIS.Leg_Data.LegCtrl[LegLeft].LegStartCnt < 500)
  {
    Set_DM8009_MotorCurrent(&JointBus,Left_Front_Tx_ID,0,0,0,0,0);
    Set_DM8009_MotorCurrent(&JointBus,Left_Back_Tx_ID,0,0,0,0,0);
    CHASSIS.Leg_Data.LegCtrl[LegLeft].LegStartCnt ++;
  }
  else if(CHASSIS.Leg_Data.LegCtrl[LegLeft].LegStartCnt >= 500)
  {
    Set_DM8009_MotorCurrent(&JointBus,Left_Front_Tx_ID,0,0,-CHASSIS.Leg_Data.LegCtrl[LegLeft].Leg_VMC.Tp1,0,0);
    Set_DM8009_MotorCurrent(&JointBus,Left_Back_Tx_ID,0,0,-CHASSIS.Leg_Data.LegCtrl[LegLeft].Leg_VMC.Tp2,0,0);
    CHASSIS.Leg_Data.LegCtrl[LegLeft].LegStartCnt = 500;
  }

  /******************** 右腿计算 ********************/
  /* 腿长拟合反馈矩阵 */
  SetLQR_K_Feedback(&CHASSIS.Leg_Data.LegCtrl[LegRight],CHASSIS.Leg_Data.LegCtrl[LegRight].Leg_VMC.L0);
  /* VMC运动学解算 */
  LegVMC_Position_Ctrl(&CHASSIS.Leg_Data.LegCtrl[LegRight]);
  /* 右腿状态反馈 */
  SetLQR_Right_State();
  /* LQR计算 */
  LQR_Ctrl(&CHASSIS.Leg_Data.LegCtrl[LegRight]);
  /* LQR虚拟关节赋值 */
  SetLQR_RightVirtualJoint();
  /* VMC动力学解算 */
  LegVMC_F_Ctrl(&CHASSIS.Leg_Data.LegCtrl[LegRight]);
  /* 关节输出 */
  if(CHASSIS.Leg_Data.LegCtrl[LegRight].LegStartCnt < 500)
  {
    Set_DM8009_MotorCurrent(&JointBus,Right_Front_Tx_ID,0,0,0,0,0);
    Set_DM8009_MotorCurrent(&JointBus,Right_Back_Tx_ID,0,0,0,0,0);
    CHASSIS.Leg_Data.LegCtrl[LegRight].LegStartCnt ++;
  }
  else if(CHASSIS.Leg_Data.LegCtrl[LegRight].LegStartCnt >= 500)
  {
    Set_DM8009_MotorCurrent(&JointBus,Right_Front_Tx_ID,0,0,CHASSIS.Leg_Data.LegCtrl[LegRight].Leg_VMC.Tp1,0,0);
    Set_DM8009_MotorCurrent(&JointBus,Right_Back_Tx_ID,0,0,CHASSIS.Leg_Data.LegCtrl[LegRight].Leg_VMC.Tp2,0,0);
    CHASSIS.Leg_Data.LegCtrl[LegRight].LegStartCnt = 500;
  }
  
  /* 转向环计算 */
  TurnAngleCtrl();
  
  /* 轮子力矩输出 */
  SetLQR_Wheel();

//  Set_DM8009_MotorCurrent(&JointBus,Left_Front_Tx_ID,0,0,0,0,0);
//  Set_DM8009_MotorCurrent(&JointBus,Left_Back_Tx_ID,0,0,0,0,0);
//  Set_DM8009_MotorCurrent(&JointBus,Right_Front_Tx_ID,0,0,0,0,0);
//  Set_DM8009_MotorCurrent(&JointBus,Right_Back_Tx_ID,0,0,0,0,0);
  //ChassisReset();

  CHASSIS.Leg_Data.LegTim = RunTimStop();
}

float Yaw_mea[2];
/**
 * @description  : 
 * @return        {*}
 */
void TurnAngleCtrl(void)
{
//  if(fabs(MasterLink.Down_Board_GetData.Spd_W) <= 1)
//  {
//    MasterLink.Down_Board_GetData.Spd_W = 0;
//  }
  
//  Yaw_mea[1] = Yaw_mea[0];
  Yaw_mea[0] = INS.Yaw + 180.0f; 

//  CHASSIS.TurnCtrl.Yaw_offset += MasterLink.Down_Board_GetData.Spd_W / 500.0f;
//  PID_Calculate(&CHASSIS.TurnCtrl.TurnPD,Yaw_mea[0],CHASSIS.TurnCtrl.Yaw_offset);
  
  if(MasterLink.Down_Board_GetData.Mode_data==Turn_Mode)
  {
    PID_Calculate(&CHASSIS.TurnCtrl.TurnPD,INS.Gyro[2],-8.0f);
  }
  else
  {
    if(fabs(MasterLink.Down_Board_GetData.Spd_W)>=1)
    {
//    CHASSIS.TurnCtrl.Yaw_offset += MasterLink.Down_Board_GetData.Spd_W / 500.0f;
      PID_Calculate(&CHASSIS.TurnCtrl.TurnPD,INS.Gyro[2],-MasterLink.Down_Board_GetData.Spd_W/5.0f);
//  CHASSIS.TurnCtrl.Yaw_offset=Yaw_mea[0];
    }
    else
    {
      PID_Calculate(&CHASSIS.TurnCtrl.TurnPD,INS.Gyro[2],0.0f);
    }
  }
//  if(Yaw_mea >= 360.0f)
//  {
//    Yaw_mea -= 360;
//  }
//  if(Yaw_mea < 0.0f)
//  {
//    Yaw_mea += 360;
//  }
  
  //CHASSIS.TurnCtrl.Yaw_offset += MasterLink.Down_Board_GetData.Spd_W / 100.0f;

//  if(CHASSIS.TurnCtrl.Yaw_offset >= 360.0f)
//  {
//    CHASSIS.TurnCtrl.Yaw_offset = CHASSIS.TurnCtrl.Yaw_offset - 360.0f;
//  }
//  if(CHASSIS.TurnCtrl.Yaw_offset < 0.0f)
//  {
//    CHASSIS.TurnCtrl.Yaw_offset += 360.0f;
//  }
  
//  if(Yaw_mea[0] - Yaw_mea[1] >= 180.0f)
//  {
//    CHASSIS.TurnCtrl.Yaw_offset += 360.0f;
//  }
//  if(Yaw_mea[0] - Yaw_mea[1] <= -180.0f)
//  {
//    CHASSIS.TurnCtrl.Yaw_offset -= 360.0f;
//  }
//  
//  if(CHASSIS.TurnCtrl.Yaw_offset - Yaw_mea[0] >= 180.0f)
//  {
//    CHASSIS.TurnCtrl.Yaw_offset -= 180.0f;
//  }
//  if(CHASSIS.TurnCtrl.Yaw_offset - Yaw_mea[0] <= -180.0f)
//  {
//    CHASSIS.TurnCtrl.Yaw_offset += 180.0f;
//  }
  
}

/**
 * @description  : 双腿协调计算
 * @param         {float} l_theta：左腿的theta值
 * @param         {float} r_theta：右腿的theta值
 * @return        {*}
 */
void LegSplitCtrl(void)
{
  CHASSIS.TunningCtrl.delta_theta=CHASSIS.Leg_Data.LegCtrl[0].LQR_Data.LQR_State.theta - CHASSIS.Leg_Data.LegCtrl[1].LQR_Data.LQR_State.theta;      //计算出两腿theta的差值
  PID_Calculate(&CHASSIS.TunningCtrl.TunningPD,CHASSIS.TunningCtrl.delta_theta,0.0f);
}

void RollCtrl(void)
{
  PID_Calculate(&CHASSIS.RollCtrl.RollPD,INS.Roll,CHASSIS.RollCtrl.RollTarget);
  
}
