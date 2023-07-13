#ifndef __MOTOR_H
#define __MOTOR_H
#include <sys.h>	 
#define PWMB   TIM3->CCR4 //B电机PWM
#define PWMA   TIM3->CCR3 //A电机PWM

#define CB1   PBout(12)  //B电机方向
#define CB2   PBout(13)  //B电机方向
#define CA1   PBout(14)  //A电机方向
#define CA2   PBout(15)  //A电机方向
void Motor_PWM_Init(u16 arr,u16 psc);
void Steering_engine_PWM_Init(u16 arr,u16 psc);
#endif
