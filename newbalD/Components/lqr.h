/*
 * @File         : 
 * @Author       : JL HUANG
 * @Date         : 2024-01-05 17:19:33
 * @LastEditTime : 2024-05-19 12:04:36
 * @FilePath     : \MDK-ARMc:\Users\JLH\Desktop\newbalD\Components\lqr.h
 * @Description  : 
 */

#ifndef __LQR_H
#define __LQR_H

#include "main.h"

/* 轮子重力 */
#define WHEEL_G  6.0f
#define WHEEL_R  0.1f

/* 反馈矩阵拟合系数 */
#define K111 -109.758189f
#define K112 169.821659f
#define K113 -122.844249f
#define K114 -3.152046f

#define K121 8.276242f
#define K122 -9.529982f
#define K123 -7.728356f
#define K124 -0.382047f

#define K131 -19.039064f
#define K132 28.443602f
#define K133 -15.288326f
#define K134 -2.624639f

#define K141 -11.669809f
#define K142 20.159292f
#define K143 -13.888823f
#define K144 -3.004050f

#define K151 -16.292408f
#define K152 54.672672f
#define K153 -54.010595f
#define K154 22.911518f

#define K161 -6.049793f
#define K162 11.077551f
#define K163 -8.290046f
#define K164 3.523985f


#define K211 285.982157f
#define K212 -316.498376f
#define K213 101.574875f
#define K214 9.284637f

#define K221 21.425217f
#define K222 -27.753675f
#define K223 12.162736f
#define K224 0.834716f

#define K231 4.857877f
#define K232 16.078099f
#define K233 -25.306610f
#define K234 11.574893f

#define K241 1.891281f
#define K242 17.341041f
#define K243 -24.350847f
#define K244 11.383404f

#define K251 194.783278f
#define K252 -279.844082f
#define K253 146.049418f
#define K254 13.857553f

#define K261 30.815665f
#define K262 -43.683439f
#define K263 22.811064f
#define K264 -0.137491f




/**
 * @description  : 状态向量结构体
 * @return        {*}
 */
typedef struct
{
  float theta_last; //保留上一次角度
  float phi_last;
  float x_last;
  
  float theta;      //rad
  float theta_dot;
  float x;   //m
  float x_dot;
  float phi;        //rad
  float phi_dot;
}LQR_State_t;

/**
 * @description  : 控制向量结构体
 * @return        {*}
 */
typedef struct
{
  float T_Wheel;
  float Tp_Joint;
}LQR_Ctrl_t;

typedef struct
{
  float K_Feedback[2][6];
  LQR_Ctrl_t LQR_Ctrl;
  LQR_State_t LQR_State;
}LQR_Data_t;

void SetLQR_LeftVirtualJoint(void);
void SetLQR_RightVirtualJoint(void);
void SetLQR_Wheel(void);
void Set_X_Average(void);

#endif
