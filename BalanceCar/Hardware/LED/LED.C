#include "led.h"

/**************************************************************************
LED接口初始化
**************************************************************************/
void LED_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE); //使能端口时钟
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;	            //端口配置
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;      //推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;     //50M
  GPIO_Init(GPIOA, &GPIO_InitStructure);					      //根据设定参数初始化GPIO
}

/**************************************************************************
LED闪烁
**************************************************************************/
void Led_Flash(u16 time)
{           
	  static int last_time,temp;
		   if(temp==0)last_time=time;
	     if(last_time==0)      LED=1;     //时间为0时设置灯状态为灭
       else if(last_time==1) LED=0;     //时间为1时设置灯状态为亮
	     else	if(++temp==last_time)	LED=~LED,temp=0;//其他非0和1的正整数设置灯状态为闪烁
	  
}
