#include "led.h"

/**************************************************************************
LED�ӿڳ�ʼ��
**************************************************************************/
void LED_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE); //ʹ�ܶ˿�ʱ��
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;	            //�˿�����
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;      //�������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;     //50M
  GPIO_Init(GPIOA, &GPIO_InitStructure);					      //�����趨������ʼ��GPIO
}

/**************************************************************************
LED��˸
**************************************************************************/
void Led_Flash(u16 time)
{           
	  static int last_time,temp;
		   if(temp==0)last_time=time;
	     if(last_time==0)      LED=1;     //ʱ��Ϊ0ʱ���õ�״̬Ϊ��
       else if(last_time==1) LED=0;     //ʱ��Ϊ1ʱ���õ�״̬Ϊ��
	     else	if(++temp==last_time)	LED=~LED,temp=0;//������0��1�����������õ�״̬Ϊ��˸
	  
}