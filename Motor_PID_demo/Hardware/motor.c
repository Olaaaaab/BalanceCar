#include "stm32f10x.h"                  // Device header
#include "pwm.h"
/*

һ���˿ڽ�PWM������һ���Ӹߵ͵�ƽ�����۽��ĸ���ƽ�����PWM���β��䣬��PWM���30
IO�Ӹ���PWM���ϰ�������ѹ��������30�ĵ�ѹ����ת
IO������PWM���ϰ�������ѹ��������70�ĵ�ѹ����ת

*/
void left_f(uint8_t speed)       //A0+B0+B12
{
    PWM_SetCompare1(speed);       
    GPIO_ResetBits(GPIOB, GPIO_Pin_0);
    GPIO_SetBits(GPIOB, GPIO_Pin_12);
}
void left_b(uint8_t speed)       //A0+B0+B12
{
    PWM_SetCompare1(speed);       
    GPIO_SetBits(GPIOB, GPIO_Pin_0);
    GPIO_ResetBits(GPIOB, GPIO_Pin_12);
}
void right_f(uint8_t speed)       //A1+B13+B14
{
    PWM_SetCompare2(speed);       
    GPIO_ResetBits(GPIOB, GPIO_Pin_15);
    GPIO_SetBits(GPIOB, GPIO_Pin_14);
}
void right_b(uint8_t speed)       //A1+B13+B14
{
    PWM_SetCompare2(speed);       
    GPIO_SetBits(GPIOB, GPIO_Pin_15);
    GPIO_ResetBits(GPIOB, GPIO_Pin_14);
}

void Motor_ENABLE(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB,&GPIO_InitStructure);
	
	GPIO_SetBits(GPIOB,GPIO_Pin_10);
}
