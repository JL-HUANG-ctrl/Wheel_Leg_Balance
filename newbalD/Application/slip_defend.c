
#include "slip_defend.h"
#include "ins_task.h"
#include "cmsis_os.h"

slip_denfend_t chassis_slip_defend;

const float detect_dt = 0.001f;

const float thegma = 0.0008f;

    static float P_Init[4] =
        {
            1.0f,
            0.0f,
            0.0f,
            1.0f,
        };

    static float F_Init[4] =
        {
            1.0f,
            detect_dt,
            0.0f,
            1.0f,
        };
    static float Q_Init[4] =
        {
            3.2,0.0f,0.f,15.0f
        };
				
			
    static float H_Init[4] =
        {
            1.0f,
            0.0f,
            0.0f,
            1.0f,
        };
				
float R_Init[4] = {800.0f, 0.0f, 
                   0.0f,  5000.0f}; 

									 
									 
void slip_defence_init(void)
{

    chassis_slip_defend.V_expect_l = 0.0f;
    chassis_slip_defend.V_expect_r = 0.0f;
	
	chassis_slip_defend.X_expect_l=0.0f;
	chassis_slip_defend.X_expect_r=0.0f;



//    static float state_min_variance[2] = {
//        0.0001f,
//        0.0001f,
//    };
//  
//    static float measurement_reference[2] = {1.0f, 1.0f};
//    static float measurement_degree[2] = {1.0f, 1.0f};

//    static float mat_R_diagonal_elements[2] = {100.0f,100.0f};

    Kalman_Filter_Init(&chassis_slip_defend.slip_defence_kf[0], 2, 0, 2);
    memcpy(chassis_slip_defend.slip_defence_kf[0].P_data, P_Init, sizeof(P_Init));
    memcpy(chassis_slip_defend.slip_defence_kf[0].F_data, F_Init, sizeof(F_Init));
    memcpy(chassis_slip_defend.slip_defence_kf[0].Q_data, Q_Init, sizeof(Q_Init));
		memcpy(chassis_slip_defend.slip_defence_kf[0].R_data, R_Init, sizeof(R_Init));
//    memcpy(chassis_slip_defend.slip_defence_kf[0].MeasurementMap, measurement_reference, sizeof(measurement_reference));
//    memcpy(chassis_slip_defend.slip_defence_kf[0].MeasurementDegree, measurement_degree, sizeof(measurement_degree));
//    memcpy(chassis_slip_defend.slip_defence_kf[0].MatR_DiagonalElements, mat_R_diagonal_elements, sizeof(mat_R_diagonal_elements));
//    memcpy(chassis_slip_defend.slip_defence_kf[0].StateMinVariance, state_min_variance, sizeof(state_min_variance));
		memcpy(chassis_slip_defend.slip_defence_kf[0].H_data,H_Init,sizeof(H_Init));
//		memcpy(chassis_slip_defend.slip_defence_kf[0].HT_data,H_Init,sizeof(H_Init));
//	chassis_slip_defend.slip_defence_kf[0].SkipEq5 = TRUE;
//		chassis_slip_defend.slip_defence_kf[0].SkipEq4=TRUE;
    Kalman_Filter_Init(&chassis_slip_defend.slip_defence_kf[1], 2, 0, 2);
    memcpy(chassis_slip_defend.slip_defence_kf[1].P_data, P_Init, sizeof(P_Init));
    memcpy(chassis_slip_defend.slip_defence_kf[1].F_data, F_Init, sizeof(F_Init));
    memcpy(chassis_slip_defend.slip_defence_kf[1].Q_data, Q_Init, sizeof(Q_Init));
		memcpy(chassis_slip_defend.slip_defence_kf[1].R_data, R_Init, sizeof(R_Init));
//    memcpy(chassis_slip_defend.slip_defence_kf[1].MeasurementMap, measurement_reference, sizeof(measurement_reference));
//    memcpy(chassis_slip_defend.slip_defence_kf[1].MeasurementDegree, measurement_degree, sizeof(measurement_degree));
//    memcpy(chassis_slip_defend.slip_defence_kf[1].MatR_DiagonalElements, mat_R_diagonal_elements, sizeof(mat_R_diagonal_elements));
//    memcpy(chassis_slip_defend.slip_defence_kf[1].StateMinVariance, state_min_variance, sizeof(state_min_variance));
		memcpy(chassis_slip_defend.slip_defence_kf[1].H_data,H_Init,sizeof(H_Init));
//		memcpy(chassis_slip_defend.slip_defence_kf[1].HT_data,H_Init,sizeof(H_Init));
//		chassis_slip_defend.slip_defence_kf[1].SkipEq5 = TRUE;
//		chassis_slip_defend.slip_defence_kf[1].SkipEq4=TRUE;
}

void slip_Defence_task(void)
{
	
		
    // 腿零的速度，对应左腿
    chassis_slip_defend.wl = INS.Gyro[0] + CHASSIS.Leg_Data.LegCtrl[0].LQR_Data.LQR_State.phi_dot -CHASSIS.MF9025_Motor[MF9025_ID_Left-1].MF9025_MotorMeasure.speed_rad;

    chassis_slip_defend.V_actual_l = chassis_slip_defend.wl * Wheel_R - chassis_L0 * CHASSIS.Leg_Data.LegCtrl[0].LQR_Data.LQR_State.theta_dot * arm_cos_f32(CHASSIS.Leg_Data.LegCtrl[0].LQR_Data.LQR_State.theta);
    // 腿一的速度，对应右腿
    chassis_slip_defend.wr = INS.Gyro[0] + CHASSIS.Leg_Data.LegCtrl[1].LQR_Data.LQR_State.phi_dot+CHASSIS.MF9025_Motor[MF9025_ID_Right-1].MF9025_MotorMeasure.speed_rad;

    chassis_slip_defend.V_actual_r = chassis_slip_defend.wr * Wheel_R + chassis_L0 * CHASSIS.Leg_Data.LegCtrl[1].LQR_Data.LQR_State.theta_dot * arm_cos_f32(CHASSIS.Leg_Data.LegCtrl[1].LQR_Data.LQR_State.theta);

    chassis_slip_defend.Accel = INS.MotionAccel_n[0];
    // 进行kalman的赋值、更新
    chassis_slip_defend.slip_defence_kf[0].MeasuredVector[0]=chassis_slip_defend.V_actual_l;
    chassis_slip_defend.slip_defence_kf[0].MeasuredVector[1] = INS.MotionAccel_n[0]-  chassis_slip_defend.accel_filter_f;

    chassis_slip_defend.slip_defence_kf[1].MeasuredVector[0] = chassis_slip_defend.V_actual_r;
    chassis_slip_defend.slip_defence_kf[1].MeasuredVector[1] = INS.MotionAccel_n[0]-chassis_slip_defend.accel_filter_f;

    Kalman_Filter_Update(&chassis_slip_defend.slip_defence_kf[0]);
    Kalman_Filter_Update(&chassis_slip_defend.slip_defence_kf[1]);

    chassis_slip_defend.V_expect_l = -chassis_slip_defend.slip_defence_kf[0].FilteredValue[0];
    chassis_slip_defend.V_expect_r = -chassis_slip_defend.slip_defence_kf[1].FilteredValue[0];
		
//		chassis_slip_defend.X_expect_l=chassis_slip_defend.X_expect_l+chassis_slip_defend.V_expect_l*0.004f;
//		chassis_slip_defend.X_expect_r=chassis_slip_defend.X_expect_r+chassis_slip_defend.V_expect_r*0.004f;
//		
        chassis_slip_defend.X_expect_l=0.0f;
        chassis_slip_defend.X_expect_r=0.0f;
		chassis_slip_defend.ACCEL_TOTAL+=INS.MotionAccel_n[0]-  0.0089f;
		
		//buffer_to_vofa(chassis_slip_defend.ACCEL_TOTAL,0,0,0);
		
		
}

void slip_kalman_update(void)
{
	
	
	
}
