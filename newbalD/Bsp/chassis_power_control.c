/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       chassis_power_control.c/h
  * @brief      chassis power control.底盘功率控制
  * @note       
  *             控制功率，主要通过控制电机电流设定值,如果限制功率是40w，减少
  *             JUDGE_TOTAL_CURRENT_LIMIT和POWER_CURRENT_LIMIT的值，还有底盘最大速度
  *             (包括max_vx_speed, min_vx_speed)
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Nov-11-2019     RM              1. add chassis power control
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */
#include "chassis_power_control.h"
#include "referee.h"
#include "arm_math.h"
#include"refereeData_v1.6.h"
#include "chassis_task.h"
#include "string.h"
#include "math.h"
//#include "detect_task.h"

extern int iq[2] ;
int myiq[2] ;
#define NO_JUDGE_TOTAL_CURRENT_LIMIT    64000.0f    //16000 * 4, 

/**
  * @brief          limit the power, mainly limit motor current
  * @param[in]      chassis_power_control: chassis data 
  * @retval         none
  */
/**
  * @brief          限制功率，主要限制电机电流
  * @param[in]      chassis_power_control: 底盘数据
  * @retval         none
  */
float POWER_LIMIT= 45.0f;//功率限制45w,动态的
float WARNING_POWER = 27.0f ;//动态的
float WARNING_POWER_BUFF = 60.0f;//，固定的，60J缓冲能量 
float BUFFER_TOTAL_CURRENT_LIMIT = 13000.0f;
float POWER_TOTAL_CURRENT_LIMIT = 11000.0f;
extern game_state_t game_state; 
extern power_heat_data_t power_heat_data; 

 float chassis_power = 0.0f;
    float chassis_power_buffer = 0.0f;
    float total_current_limit = 0.0f;
    float total_current = 0.0f;
    
void chassis_power_control()
{
//    float chassis_power = 0.0f;
//    float chassis_power_buffer = 0.0f;
//    float total_current_limit = 0.0f;
//    float total_current = 0.0f;
	// if(0==(chassis_power=cap_power_heat_data_t.cap_output_power))
	// {
		POWER_LIMIT=robot_state.chassis_power_limit;//获得底盘功率上限
		chassis_power=power_heat_data.chassis_power;
	// }
	// else
	// {
	// 	POWER_LIMIT=robot_state.chassis_power_limit+30;
	// }
	WARNING_POWER=POWER_LIMIT*2.0f/3.0f;
	//下面两行是根据底盘功率上限调整电流上限
	BUFFER_TOTAL_CURRENT_LIMIT=10000.0f* (POWER_LIMIT/40.0f);
	POWER_TOTAL_CURRENT_LIMIT=10000.0f*(POWER_LIMIT/40.0f);
	{
			chassis_power_buffer= power_heat_data.buffer_energy;
        //功率超过上限功率 和缓冲能量小于60j,因为缓冲能量小于60意味着功率超过上限功率
        if(chassis_power_buffer < WARNING_POWER_BUFF)
        {//超功率了，进入这里
            float power_scale;
            if(chassis_power_buffer > 5.0f)
            {
                //缩小WARNING_POWER_BUFF
                power_scale = chassis_power_buffer / WARNING_POWER_BUFF;
            }
            else
            {//进入这里，说明缓冲能量即将耗尽，要扣血死亡了
                //only left 10% of WARNING_POWER_BUFF
                power_scale = 5.0f / WARNING_POWER_BUFF;
            }
            //缩小
            total_current_limit = BUFFER_TOTAL_CURRENT_LIMIT * power_scale;
        }
        else
        {
            //功率大于WARNING_POWER
            if(chassis_power > WARNING_POWER)
            {//可能马上超功率，也可能已经超功率了
                float power_scale;
                //功率小于60w
                if(chassis_power < POWER_LIMIT)
                {//马上超功率了
                    //缩小
                    power_scale = (POWER_LIMIT - chassis_power) / (POWER_LIMIT - WARNING_POWER);
                    
                }
                else
                {//已经超功率了
                    power_scale = 0.0f;
                }
                
                total_current_limit = BUFFER_TOTAL_CURRENT_LIMIT + POWER_TOTAL_CURRENT_LIMIT * power_scale;
            }
            //功率小于WARNING_POWER
            else
            {//没有马上超功率
                total_current_limit = BUFFER_TOTAL_CURRENT_LIMIT + POWER_TOTAL_CURRENT_LIMIT;
            }
        }
    } 
    total_current = 0.0f;
    //计算原本电机电流设定
    for(uint8_t i = 0; i < 2; i++)
    {
        total_current += (float)abs(iq[i]);
    }
    
    if(total_current > total_current_limit)
    {
        float current_scale = total_current_limit / total_current;
        myiq[0]=iq[0];
       myiq[1]=iq[1];
        iq[0]=(int)(iq[0]*current_scale);
        iq[1]=(int)(iq[1]*current_scale);
    }
}




