#include "stm32f10x.h"                  // Device header
#include "Delay.h"
#include "OLED.h"
#include "serial.h"
#include "pwm.h"
#include "motor.h"
#include "stdio.h"
#include  "ENCODER.H"
#include "PID.H"
#include "led.h"

int16_t speed,time=0,tar=60;

int main(void)
{
	OLED_Init();
    LED_Init();
    Serial_Init();
    PWM_Init();
	Motor_ENABLE();
    Encoder1_Init();
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	
    OLED_ShowString(1, 1, "speed:");
	int outspeed;

	while (1)
	{
		outspeed = pid1(speed,tar);
        left_f(outspeed);
        OLED_ShowNum(1,7,outspeed,4);
	}
}

void TIM2_IRQHandler(void)
{
	if (TIM_GetITStatus(TIM2, TIM_IT_Update) == SET)
	{
        time++;
        if(time==10)
        {
            speed=Encoder1_get();
            time=0;
        }
		TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
	}
}






