

#ifndef _SLIP_DEFEND_H_
#define _SLIP_DEFEND_H_

#include "main.h"
#include "kalman_filter.h"
#include "FreeRTOS.h"
#include "task.h"
#include "chassis_task.h"

#include "ins_task.h"

#define Wheel_R 0.0875f
#define chassis_L0 0.14f

typedef struct
{
	float V_expect_l;
	float V_expect_r;
	
	float V_actual_l;
	float V_actual_r;
	
	float X_expect_l;
	float X_expect_r;
	
	float Accel;
	float ACCEL_TOTAL;
	
	KalmanFilter_t slip_defence_kf[2];
	
	float wl;
	float wr;
	float accel_filter_f;
	
}slip_denfend_t;

 
extern slip_denfend_t chassis_slip_defend;


void slip_defence_init(void);
void slip_Defence_task(void);

#endif // _SLIP_DEFEND_H_


