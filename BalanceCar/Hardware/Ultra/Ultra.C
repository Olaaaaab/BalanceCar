#include "Ultra.h"
u16 TIM1CH1_CAPTURE_STA,TIM1CH1_CAPTURE_VAL;
/**************************************************************************
函数功能：定时器1通道1输入捕获初始化

**************************************************************************/
void TIM1_Cap_Init(u16 arr,u16 psc)	
{	 
  GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);//开启引脚时钟	
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;         //端口配置PA8     
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;     //引脚设置为上拉输入
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&GPIO_InitStructure);
	
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;	            //端口配置PA11
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;      //推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;     //50M
  GPIO_Init(GPIOA, &GPIO_InitStructure);					      //根据设定参数初始化GPIO

/******************************************/	
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_ICInitTypeDef TIM1_ICInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE); //TIM1 时钟使能
 
	TIM_ClearITPendingBit(TIM1, TIM_IT_Update );     //清除TIM1更新中断标志 
 
	//定时器 TIM1 初始化
	TIM_TimeBaseStructure.TIM_Period = arr; //设置自动重装载寄存器的周期值，
	TIM_TimeBaseStructure.TIM_Prescaler = psc; //设置预分频值
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //设置时钟分频系数
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; //TIM 向上计数
	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure); //初始化 TIM1
 
	//TIM1输入捕获参数配置
	TIM1_ICInitStructure.TIM_Channel = TIM_Channel_1; //捕获通道IC1
	TIM1_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising; //上升沿捕获
	TIM1_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //直接映射
	TIM1_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1; //不分频，每个变化沿都捕获
	TIM1_ICInitStructure.TIM_ICFilter = 0x00;//不滤波
	TIM_ICInit(TIM1, &TIM1_ICInitStructure);
 
	//中断优先级 NVIC 配置
	NVIC_InitStructure.NVIC_IRQChannel = TIM1_CC_IRQn; //TIM1 捕获中断
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1; //抢占优先级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1; //从优先级
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQ 通道使能
	NVIC_Init(&NVIC_InitStructure); //初始化 NVIC 寄存器
 
	TIM_ITConfig(TIM1,TIM_IT_Update|TIM_IT_CC1,ENABLE);       //使能更新中断和捕获中断
 
	TIM_Cmd(TIM1, ENABLE);         //使能定时器
}

/**************************************************************************
函数功能:发射声波，获取回波
**************************************************************************/
int distance;
int Get_distance(void)
{   
 
		   PAout(11)=1;     //发射控制端给高电平      
	     delay_us(20);    //高电平持续时间
	     PAout(11)=0;     //发射控制端电平置低
		if(TIM1CH1_CAPTURE_STA&0X80)//成功捕获到了一次高电平
		{
			distance=TIM1CH1_CAPTURE_STA&0X3F;
			distance*=65536;					    //溢出时间总和
			distance+=TIM1CH1_CAPTURE_VAL;		//得到总的高电平时间
			distance=distance*170/1000;           //距离=接收到高电平持续时间*音速/2（1000是音速的时间单位换算成毫秒）				                       
			TIM1CH1_CAPTURE_STA=0;			    //开启下一次捕获
		}	
		return distance;
}
/**************************************************************************
函数功能：超声波回波脉宽读取中断
**************************************************************************/

void TIM1_CC_IRQHandler(void)
{ 		    		  			    
	u16 tsr;
	tsr=TIM1->SR;
	if((TIM1CH1_CAPTURE_STA&0X80)==0)//还未成功捕获	
				                    {
                                 	 if(tsr&0X01)//溢出
					                            {	    
						                         if(TIM1CH1_CAPTURE_STA&0X40)//已经捕获到高电平了
							                       {
								                    if((TIM1CH1_CAPTURE_STA&0X3F)==0X3F)//高电平太长了
									                  {
										               TIM1CH1_CAPTURE_STA|=0X80;//标记成功捕获了一次
										               TIM1CH1_CAPTURE_VAL=65535;
									                  }
								                    else TIM1CH1_CAPTURE_STA++;
							                       }	 
					                             }
				                     if(tsr&0x02)//捕获1发生捕获事件
				    	                        {	
						                         if(TIM1CH1_CAPTURE_STA&0X40)		//捕获到一个下降沿 		
							                       {			
								                    TIM1CH1_CAPTURE_STA|=0X80;		//标记成功捕获到一次高电平脉宽
								                    TIM1CH1_CAPTURE_VAL=TIM1->CCR1;	//获取当前的捕获值.
								                    TIM1->CCER&=~(1<<1);		//CC1P=0 设置为上升沿捕获
							                       }
						                         else  //还未开始,第一次捕获上升沿
				   	                               {         
								                    TIM1CH1_CAPTURE_STA=0;		//清空
								                    TIM1CH1_CAPTURE_VAL=0;     //清空
								                    TIM1CH1_CAPTURE_STA|=0X40;		//标记捕获到了上升沿
								                    TIM1->CNT=0;			//计数器清空
								                    TIM1->CCER|=1<<1; 			//CC1P=1 设置为下降沿捕获
							                       }		    
							                     }			     	    					   
		                              }
	TIM1->SR=0;//清除中断标志位 	     
}
