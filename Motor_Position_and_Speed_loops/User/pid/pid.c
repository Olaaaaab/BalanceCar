#include "./pid/pid.h"

g_tPID PID;

void PID_INIT(void)
{
	PID.f_Actual_val = 0.0;
	PID.f_Target_val = 0.0;
	PID.f_Err = 0.0;
	PID.f_Err_last = 0.0;
	PID.f_Integral = 0.0;
	PID.f_Kd = 0.0;
	PID.f_Ki = 0.0;
	PID.f_Kp = 0.0;
}

void Set_Target_Val(float target_speed)
{
	PID.f_Target_val = target_speed;
}

void Get_Current_Val(float real_vel)
{
	PID.f_Actual_val = real_vel;
}


void Set_Kp_Ki_Kd(float Kp, float Ki, float Kd)
{
  	PID.f_Kp = Kp;    // 设置比例系数 P
	PID.f_Ki = Ki;    // 设置积分系数 I
	PID.f_Kd = Kd;    // 设置微分系数 D
}

float PID_CONTORL(float target_speed,float real_vel, float p, float i, float d)
{
	float output_val = 0;

	Set_Target_Val(target_speed);
	Get_Current_Val(real_vel);
	Set_Kp_Ki_Kd(p, i, d);
	
	PID.f_Err = PID.f_Target_val - PID.f_Actual_val;
	PID.f_Integral += PID.f_Err;
	
	
	output_val = (PID.f_Kp * PID.f_Err) + (PID.f_Ki * PID.f_Integral) + (PID.f_Kd * (PID.f_Err - PID.f_Err_last));
		
	PID.f_Err_last = PID.f_Err;
	return output_val;
}

float PID_POS_CONTORL(int excpet_count ,int capture_count, float p, float i, float d)
{
	float output_val = 0;

	Set_Target_Val(excpet_count);
	Get_Current_Val(capture_count);
	Set_Kp_Ki_Kd(p, i, d);
	
	PID.f_Err = PID.f_Target_val - PID.f_Actual_val;
	PID.f_Integral += PID.f_Err;
	
	output_val = (PID.f_Kp * PID.f_Err) + (PID.f_Ki * PID.f_Integral) + (PID.f_Kd * (PID.f_Err - PID.f_Err_last));
	
	PID.f_Err_last = PID.f_Err;

	return output_val;
}
