#include "usart.h"	  
//�������´���,֧��printf����,������Ҫѡ��use MicroLIB	  
#if 1
#pragma import(__use_no_semihosting)             
//��׼����Ҫ��֧�ֺ���                 
struct __FILE 
{ 
	int handle; 
	/* Whatever you require here. If the only file you are using is */ 
	/* standard output using printf() for debugging, no file handling */ 
	/* is required. */ 
}; 
/* FILE is typedef�� d in stdio.h. */ 
FILE __stdout;       
//����_sys_exit()�Ա���ʹ�ð�����ģʽ    
int _sys_exit(int x) 
{ 
	x = x; 
	return 0;
} 
//�ض���fputc���� 
int fputc(int ch, FILE *f)
{      

	while((USART3->SR&0X40)==0);
	USART3->DR = (u8) ch;      
  return ch;
}
#endif 

//******************************����1******************************************//
/////////////////////////////////////////////////////////////////////////////////
//****************usart1����һ���ֽ�************************************//
void usart1_send(u8 data)
{
	USART1->DR = data;
	while((USART1->SR&0x40)==0);	
}
//****************����1��ʼ��************************************//
void usart1_init(u32 bound){
  //GPIO�˿�����
  GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1|RCC_APB2Periph_GPIOA, ENABLE);	//ʹ��USART1��GPIOAʱ��
  
	//USART1_TX   GPIOA.9
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9; //PA.9
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//�����������
  GPIO_Init(GPIOA, &GPIO_InitStructure);//��ʼ��GPIOA.9
   
  //USART1_RX	  GPIOA.10��ʼ��
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;//PA10
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//��������
  GPIO_Init(GPIOA, &GPIO_InitStructure);//��ʼ��GPIOA.10  
   //USART ��ʼ������

	USART_InitStructure.USART_BaudRate = bound;//���ڲ�����
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//�ֳ�Ϊ8λ���ݸ�ʽ
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//һ��ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No;//����żУ��λ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//�շ�ģʽ

  USART_Init(USART1, &USART_InitStructure); //��ʼ������1
  USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);//�������ڽ����ж�
  USART_Cmd(USART1, ENABLE);                    //ʹ�ܴ���1 

}

//******************************����1�����ж�*************************************//
int USART1_IRQHandler(void)
{	
	if(USART1->SR&(1<<5))//���յ�����
	{	      

   }
return 0;	
}
//*******************������λ��v4.22������������************************//
void usart1_send_char(u8 c)
{
    while((USART1->SR&0X40)==0);//�ȴ���һ�η������  
    USART1->DR=c;   
}
 
//fun:������. 0XA0~0XAF
//data:���ݻ�����,���28�ֽ�!!
//len:data����Ч���ݸ���
void usart1_niming_report(u8 fun,u8*data,u8 len)
{
    u8 send_buf[32];
    u8 i;
    if(len>28)return;    //���28�ֽ�����
    send_buf[len+4]=0;  //У��������
    send_buf[0]=0XAA;   //֡ͷ
	  send_buf[1]=0XAA;   //֡ͷ
    send_buf[2]=fun;    //������
    send_buf[3]=len;    //���ݳ���
    for(i=0;i<len;i++)send_buf[4+i]=data[i];         //��������
    for(i=0;i<len+4;i++)send_buf[len+4]+=send_buf[i];    //����У���
    for(i=0;i<len+5;i++)usart1_send_char(send_buf[i]);   //�������ݵ�����1
}
//************************��������Ƕ�**************************// 
void mpu6050_send_data(float pitch_,float roll_,float yaw_)
{
    u8 tbuf[16];
    unsigned char *p;
    p=(unsigned char *)&pitch_;
    tbuf[0]=(unsigned char)(*(p+3));
    tbuf[1]=(unsigned char)(*(p+2));
    tbuf[2]=(unsigned char)(*(p+1));
    tbuf[3]=(unsigned char)(*(p+0));
     
    p=(unsigned char *)&roll_;
    tbuf[4]=(unsigned char)(*(p+3));
    tbuf[5]=(unsigned char)(*(p+2));
    tbuf[6]=(unsigned char)(*(p+1));
    tbuf[7]=(unsigned char)(*(p+0));
     
    p=(unsigned char *)&yaw_;
    tbuf[8]=(unsigned char)(*(p+3));
    tbuf[9]=(unsigned char)(*(p+2));
    tbuf[10]=(unsigned char)(*(p+1));
    tbuf[11]=(unsigned char)(*(p+0));
     
    usart1_niming_report(0XF1,tbuf,12);//�Զ���֡,������0XF1
}  


//******************************����2********************************//
///////////////////////////////////////////////////////////////////////
//****************************usart2����һ���ֽ�*********************//
void usart2_send(u8 data)
{
	USART2->DR = data;
	while((USART2->SR&0x40)==0);	
}
//******************************����2��ʼ��**************************//
void usart2_init(u32 bound)
{  	 
  GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
 
	RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOA, ENABLE);	//ʹ��GPIOAʱ��
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);	//ʹ��USARTʱ��
	//USART2_TX   GPIOA.2
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2; //PA.2
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//�����������
  GPIO_Init(GPIOA, &GPIO_InitStructure);//��ʼ��GPIOA.2
   
  //USART2_RX	  GPIOA.3��ʼ��
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;//PA3
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//��������
  GPIO_Init(GPIOA, &GPIO_InitStructure);//��ʼ��GPIOA.3  
   //USART ��ʼ������

	  //UsartNVIC ����
  NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0 ;//��ռ���ȼ�
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;		//�����ȼ�
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ���
	
	USART_InitStructure.USART_BaudRate = bound;//���ڲ�����
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//�ֳ�Ϊ8λ���ݸ�ʽ
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//һ��ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No;//����żУ��λ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//�շ�ģʽ

  USART_Init(USART2, &USART_InitStructure); //��ʼ������2
  USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);//�������ڽ����ж�
  USART_Cmd(USART2, ENABLE);                    //ʹ�ܴ���2 
}

//***************************��ҡAPPͨѶ*******************//

void LANYAO_APP(int data)//�������ݵ���ҡAPP
{
  static int SEND_DATA[10];	
  int i,j,k,data_c;
if(data==0)usart2_send(0x30);
if(data>0)
	{	
	 i=data;
   while(i)
          {
	        	i=i/10;		
		        j++;
	        }	
			for(k=0;k<j;k++){SEND_DATA[k]=data%10;data=data/10;}
      for(k=j-1;k>=0;k--){usart2_send(SEND_DATA[k]+0x30);}			
   }
	if(data<0)
	{
   data_c=-data; 		
	 i=-data;
   while(i)
          {
	        	i=i/10;		
		        j++;
	        }
   usart2_send(0x2D);				
			for(k=0;k<j;k++){SEND_DATA[k]=data_c%10;data_c=data_c/10;}
      for(k=j-1;k>=0;k--){usart2_send(SEND_DATA[k]+0x30);}			
   }
	usart2_send(0x0A),
	usart2_send(0x0D);
  memset(SEND_DATA, 0, sizeof(int)*10);	
} 
//**************************����2�����ж�***********************//
int Usart2_Receive;
int data_app[10];
u8 flag_mode_app=0;
int anjian_app,huakuai_app,yaogan_app=510;


int USART2_IRQHandler(void)
{	

	if(USART2->SR&(1<<5))//���յ�����
	{
	
		      Usart2_Receive=USART2->DR;
// static u8 i=0;			
//					data_app[i] = USART2->DR;//��ȡ����		                              
//			switch(i)
//			{
//				case 0:
//					if( data_app[0]==0X79)//����֡ͷ����
//					{
//						i++;//��һ���ֽ�����
//					}
//					break;
//				case 1:
//					if( data_app[1]==0X62)//����֡ͷ����
//					{
//						i++;//��һ���ֽ�����
//						flag_mode_app=0;//��������
//					}
//			   	else	if( data_app[1]==0X76)//����֡ͷ����
//					{
//						i++;//��һ���ֽ�����
//						flag_mode_app=1;//��������
//					}
//					else	if( data_app[1]==0X64)//����֡ͷ����
//					{
//						i++;//��һ���ֽ�����
//						flag_mode_app=2;//ҡ������
//					}
//					break;				
//				case 2:
//						i++;//��һ���ֽ�����
//					break;	
//				case 3:            
//            i++;//��һ���ֽ�����	
//					break;
//				case 4:
//					 if(flag_mode_app==0&&data_app[3]==0X0A&&data_app[4]==0X0D)
//						 anjian_app=data_app[2],i=0;//һ֡�������
//					 if(flag_mode_app==1&&data_app[3]==0X0A&&data_app[4]==0X0D)
//						 huakuai_app=data_app[2],i=0;//һ֡�������
//					 if(flag_mode_app==2)i++;
//					break;
//				case 5:
//					 if(data_app[4]==0X0A&&data_app[5]==0X0D)
//					 yaogan_app= data_app[2]+data_app[3],i=0;//һ֡�������				
//					break;
//				default:
//					break;				
//			}
   }
return 0;	

}
//*******************����3��ʼ��,bound:������************************//
void usart3_init(u32 bound)
{  	 
	  //GPIO�˿�����
  GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);// ��Ҫʹ��AFIOʱ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);	//ʹ��GPIOʱ��
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);	//ʹ��USARTʱ��
//		GPIO_PinRemapConfig(GPIO_PartialRemap_USART3, ENABLE);//������ӳ��
	//USART_TX  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10; //PB10
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//�����������
  GPIO_Init(GPIOB, &GPIO_InitStructure);   
  //USART_RX	  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;//PB11
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//��������
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  //UsartNVIC ����
  NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0 ;//��ռ���ȼ�
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;		//�����ȼ�
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ���
   //USART ��ʼ������
	USART_InitStructure.USART_BaudRate = bound;//���ڲ�����
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//�ֳ�Ϊ8λ���ݸ�ʽ
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//һ��ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No;//����żУ��λ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//�շ�ģʽ
  USART_Init(USART3, &USART_InitStructure);     //��ʼ������3
  USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);//�������ڽ����ж�
  USART_Cmd(USART3, ENABLE);                    //ʹ�ܴ���3 
}

//****************************�������ܣ�����3�����ж�***************************************//
int Usart3_Receive;
int USART3_IRQHandler(void)
{	

	if(USART3->SR&(1<<5))//���յ�����
	{	      
//		      Usart3_Receive=USART3->DR;
		
static u8 i=0;			
					data_app[i] = USART3->DR;//��ȡ����		                              
			switch(i)
			{
				case 0:
					if( data_app[0]==0X79)//����֡ͷ����
					{
						i++;//��һ���ֽ�����
					}
					break;
				case 1:
					if( data_app[1]==0X62)//����֡ͷ����
					{
						i++;//��һ���ֽ�����
						flag_mode_app=0;//��������
					}
			   	else	if( data_app[1]==0X76)//����֡ͷ����
					{
						i++;//��һ���ֽ�����
						flag_mode_app=1;//��������
					}
					else	if( data_app[1]==0X64)//����֡ͷ����
					{
						i++;//��һ���ֽ�����
						flag_mode_app=2;//ҡ������
					}
					break;				
				case 2:
						i++;//��һ���ֽ�����
					break;	
				case 3:            
            i++;//��һ���ֽ�����	
					break;
				case 4:
					 if(flag_mode_app==0&&data_app[3]==0X0A&&data_app[4]==0X0D)
						 anjian_app=data_app[2],i=0;//һ֡�������
					 if(flag_mode_app==1&&data_app[3]==0X0A&&data_app[4]==0X0D)
						 huakuai_app=data_app[2],i=0;//һ֡�������
					 if(flag_mode_app==2)i++;
					break;
				case 5:
					 if(data_app[4]==0X0A&&data_app[5]==0X0D)
					 yaogan_app= data_app[2]+data_app[3],i=0;//һ֡�������				
					break;
				default:
					break;				
			}
   }
return 0;	
}
