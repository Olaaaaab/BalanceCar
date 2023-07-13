#include "stm32f10x.h"                  // Device header

float Err=0,last_err=0,next_err=0,pwm=0,add=0,Kp=3.5,Ki=0.4,Kd=0;

int16_t myabs(int a)
{ 		   
	  int temp;
		if(a<0)  temp=-a;  
	  else temp=a;
	  return temp;
}

void pwm_control()
{
    if(pwm>99)
        pwm=99;
    if(pwm<0)
        pwm=0;
}

float pid1(int16_t speed1,float tar1)
{
    speed1=myabs(speed1);
    Err=tar1-speed1;
    add=Kp*(Err-last_err)+Ki*(Err)+Kd*(Err+next_err-2*last_err);
    pwm+=add;
    pwm_control();
    next_err=last_err;
    last_err=Err;
    return pwm;
}
