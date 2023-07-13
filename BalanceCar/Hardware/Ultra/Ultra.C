#include "Ultra.h"
u16 TIM1CH1_CAPTURE_STA,TIM1CH1_CAPTURE_VAL;
/**************************************************************************
�������ܣ���ʱ��1ͨ��1���벶���ʼ��

**************************************************************************/
void TIM1_Cap_Init(u16 arr,u16 psc)	
{	 
  GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);//��������ʱ��	
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;         //�˿�����PA8     
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;     //��������Ϊ��������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&GPIO_InitStructure);
	
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;	            //�˿�����PA11
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;      //�������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;     //50M
  GPIO_Init(GPIOA, &GPIO_InitStructure);					      //�����趨������ʼ��GPIO

/******************************************/	
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_ICInitTypeDef TIM1_ICInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE); //TIM1 ʱ��ʹ��
 
	TIM_ClearITPendingBit(TIM1, TIM_IT_Update );     //���TIM1�����жϱ�־ 
 
	//��ʱ�� TIM1 ��ʼ��
	TIM_TimeBaseStructure.TIM_Period = arr; //�����Զ���װ�ؼĴ���������ֵ��
	TIM_TimeBaseStructure.TIM_Prescaler = psc; //����Ԥ��Ƶֵ
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //����ʱ�ӷ�Ƶϵ��
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; //TIM ���ϼ���
	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure); //��ʼ�� TIM1
 
	//TIM1���벶���������
	TIM1_ICInitStructure.TIM_Channel = TIM_Channel_1; //����ͨ��IC1
	TIM1_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising; //�����ز���
	TIM1_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //ֱ��ӳ��
	TIM1_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1; //����Ƶ��ÿ���仯�ض�����
	TIM1_ICInitStructure.TIM_ICFilter = 0x00;//���˲�
	TIM_ICInit(TIM1, &TIM1_ICInitStructure);
 
	//�ж����ȼ� NVIC ����
	NVIC_InitStructure.NVIC_IRQChannel = TIM1_CC_IRQn; //TIM1 �����ж�
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1; //��ռ���ȼ�
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1; //�����ȼ�
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQ ͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure); //��ʼ�� NVIC �Ĵ���
 
	TIM_ITConfig(TIM1,TIM_IT_Update|TIM_IT_CC1,ENABLE);       //ʹ�ܸ����жϺͲ����ж�
 
	TIM_Cmd(TIM1, ENABLE);         //ʹ�ܶ�ʱ��
}

/**************************************************************************
��������:������������ȡ�ز�
**************************************************************************/
int distance;
int Get_distance(void)
{   
 
		   PAout(11)=1;     //������ƶ˸��ߵ�ƽ      
	     delay_us(20);    //�ߵ�ƽ����ʱ��
	     PAout(11)=0;     //������ƶ˵�ƽ�õ�
		if(TIM1CH1_CAPTURE_STA&0X80)//�ɹ�������һ�θߵ�ƽ
		{
			distance=TIM1CH1_CAPTURE_STA&0X3F;
			distance*=65536;					    //���ʱ���ܺ�
			distance+=TIM1CH1_CAPTURE_VAL;		//�õ��ܵĸߵ�ƽʱ��
			distance=distance*170/1000;           //����=���յ��ߵ�ƽ����ʱ��*����/2��1000�����ٵ�ʱ�䵥λ����ɺ��룩				                       
			TIM1CH1_CAPTURE_STA=0;			    //������һ�β���
		}	
		return distance;
}
/**************************************************************************
�������ܣ��������ز�������ȡ�ж�
**************************************************************************/

void TIM1_CC_IRQHandler(void)
{ 		    		  			    
	u16 tsr;
	tsr=TIM1->SR;
	if((TIM1CH1_CAPTURE_STA&0X80)==0)//��δ�ɹ�����	
				                    {
                                 	 if(tsr&0X01)//���
					                            {	    
						                         if(TIM1CH1_CAPTURE_STA&0X40)//�Ѿ����񵽸ߵ�ƽ��
							                       {
								                    if((TIM1CH1_CAPTURE_STA&0X3F)==0X3F)//�ߵ�ƽ̫����
									                  {
										               TIM1CH1_CAPTURE_STA|=0X80;//��ǳɹ�������һ��
										               TIM1CH1_CAPTURE_VAL=65535;
									                  }
								                    else TIM1CH1_CAPTURE_STA++;
							                       }	 
					                             }
				                     if(tsr&0x02)//����1���������¼�
				    	                        {	
						                         if(TIM1CH1_CAPTURE_STA&0X40)		//����һ���½��� 		
							                       {			
								                    TIM1CH1_CAPTURE_STA|=0X80;		//��ǳɹ�����һ�θߵ�ƽ����
								                    TIM1CH1_CAPTURE_VAL=TIM1->CCR1;	//��ȡ��ǰ�Ĳ���ֵ.
								                    TIM1->CCER&=~(1<<1);		//CC1P=0 ����Ϊ�����ز���
							                       }
						                         else  //��δ��ʼ,��һ�β���������
				   	                               {         
								                    TIM1CH1_CAPTURE_STA=0;		//���
								                    TIM1CH1_CAPTURE_VAL=0;     //���
								                    TIM1CH1_CAPTURE_STA|=0X40;		//��ǲ�����������
								                    TIM1->CNT=0;			//���������
								                    TIM1->CCER|=1<<1; 			//CC1P=1 ����Ϊ�½��ز���
							                       }		    
							                     }			     	    					   
		                              }
	TIM1->SR=0;//����жϱ�־λ 	     
}