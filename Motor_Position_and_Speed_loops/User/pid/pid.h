#ifndef __PID_H
#define	__PID_H

#include "stm32f1xx.h"
#include "./usart/bsp_debug_usart.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "./tim/bsp_motor_tim.h"
#include "main.h"

typedef struct 
{
	__IO float f_Actual_val;
	__IO float f_Target_val;
	__IO float f_Err;
	__IO float f_Err_last;
	__IO float f_Kp,f_Ki,f_Kd;
	__IO float f_Integral;
}g_tPID,g_ptPID;


void PID_INIT(void);
void Set_Target_Val(float target_speed);
void Get_Current_Val(float actual_val);
void Set_Kp_Ki_Kd(float Kp, float Ki, float Kd);
float PID_CONTORL(float target_speed,float actual_val, float p, float i, float d);
float PID_POS_CONTORL(int excpet_count ,int capture_count, float p, float i, float d);









#endif
