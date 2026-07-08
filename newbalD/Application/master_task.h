/*
 * @File         : 
 * @Author       : JL HUANG
 * @Date         : 2024-04-20 20:49:48
 * @LastEditTime : 2024-05-09 14:51:53
 * @FilePath     : \MDK-ARMc:\Users\JLH\Desktop\newbalD\Application\master_task.h
 * @Description  : 
 */

#ifndef __MASTER_TASK_H
#define __MASTER_TASK_H

#include "main.h"

#include "app_link.h"

#define STOP_Log  0 //停止完成
#define START_Log 1 //初始化完成

#define START_SYS 2 //正在初始化
#define START_END 3
#define STOP_SYS  4 //正在停止
#define STOP_END  5

#define Turn_Mode 1
#define Control_Mode 0



typedef struct
{
  Down_Board_Get_t  Down_Board_GetData;
  Down_Board_Set_t  Down_Board_SetData;
} master_link_t;

extern master_link_t MasterLink;

void MasterLink_Task(void);

#endif
