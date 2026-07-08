/*
 * @File         : 
 * @Author       : JL HUANG
 * @Date         : 2023-12-31 12:41:57
 * @LastEditTime : 2024-05-19 13:41:31
 * @FilePath     : \MDK-ARMc:\Users\JLH\Desktop\newbalD\Components\vmc.h
 * @Description  : 
 */

#ifndef __VMC_H
#define __VMC_H

#include "main.h"
#include "controller.h"


#define SUPPORT_MAX_OUT 300

typedef struct
{
  float phi1;
  float phi4;
  
  float phi2;
  float phi3;

  /* 轮子坐标 */
  float X_C;
  float Y_C;
  float L0;    //实际腿长
  float L0_dot;
  float L0_last;
  float phi0; //实际腿角度
  float phi0_last;
  float phi0_dot;

  /* 腿部推力 */
  float F;

  /* 用于离地检测的F */
  float F0;

  /* VMC输出力矩 */
  float Tp1;
  float Tp2;
}Leg_VMC_t;



void LegVMC_Init(void);
void SetLegData(void);
void SetSupport_F_Data(float kp0,float ki0,float kd0,float kp1,float ki1,float kd1);
void SetLegLen_Init(void);

#endif
