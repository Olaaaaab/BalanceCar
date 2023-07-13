#include "usart.h"	  
//加入以下代码,支持printf函数,而不需要选择use MicroLIB	  
#if 1
#pragma import(__use_no_semihosting)             
//标准库需要的支持函数                 
struct __FILE 
{ 
	int handle; 
	/* Whatever you require here. If the only file you are using is */ 
	/* standard output using printf() for debugging, no file handling */ 
	/* is required. */ 
}; 
/* FILE is typedef’ d in stdio.h. */ 
FILE __stdout;       
//定义_sys_exit()以避免使用半主机模式    
int _sys_exit(int x) 
{ 
	x = x; 
	return 0;
} 
//重定义fputc函数 
int fputc(int ch, FILE *f)
{      

	while((USART3->SR&0X40)==0);
	USART3->DR = (u8) ch;      
  return ch;
}
#endif 

//******************************串口1******************************************//
/////////////////////////////////////////////////////////////////////////////////
//****************usart1发送一个字节************************************//
void usart1_send(u8 data)
{
	USART1->DR = data;
	while((USART1->SR&0x40)==0);	
}
//****************串口1初始化************************************//
void usart1_init(u32 bound){
  //GPIO端口设置
  GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1|RCC_APB2Periph_GPIOA, ENABLE);	//使能USART1，GPIOA时钟
  
	//USART1_TX   GPIOA.9
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9; //PA.9
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//复用推挽输出
  GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化GPIOA.9
   
  //USART1_RX	  GPIOA.10初始化
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;//PA10
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//浮空输入
  GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化GPIOA.10  
   //USART 初始化设置

	USART_InitStructure.USART_BaudRate = bound;//串口波特率
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式

  USART_Init(USART1, &USART_InitStructure); //初始化串口1
  USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);//开启串口接受中断
  USART_Cmd(USART1, ENABLE);                    //使能串口1 

}

//******************************串口1接收中断*************************************//
int USART1_IRQHandler(void)
{	
	if(USART1->SR&(1<<5))//接收到数据
	{	      

   }
return 0;	
}
//*******************匿名上位机v4.22发送数据例程************************//
void usart1_send_char(u8 c)
{
    while((USART1->SR&0X40)==0);//等待上一次发送完毕  
    USART1->DR=c;   
}
 
//fun:功能字. 0XA0~0XAF
//data:数据缓存区,最多28字节!!
//len:data区有效数据个数
void usart1_niming_report(u8 fun,u8*data,u8 len)
{
    u8 send_buf[32];
    u8 i;
    if(len>28)return;    //最多28字节数据
    send_buf[len+4]=0;  //校验数置零
    send_buf[0]=0XAA;   //帧头
	  send_buf[1]=0XAA;   //帧头
    send_buf[2]=fun;    //功能字
    send_buf[3]=len;    //数据长度
    for(i=0;i<len;i++)send_buf[4+i]=data[i];         //复制数据
    for(i=0;i<len+4;i++)send_buf[len+4]+=send_buf[i];    //计算校验和
    for(i=0;i<len+5;i++)usart1_send_char(send_buf[i]);   //发送数据到串口1
}
//************************发送三轴角度**************************// 
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
     
    usart1_niming_report(0XF1,tbuf,12);//自定义帧,功能码0XF1
}  


//******************************串口2********************************//
///////////////////////////////////////////////////////////////////////
//****************************usart2发送一个字节*********************//
void usart2_send(u8 data)
{
	USART2->DR = data;
	while((USART2->SR&0x40)==0);	
}
//******************************串口2初始化**************************//
void usart2_init(u32 bound)
{  	 
  GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
 
	RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOA, ENABLE);	//使能GPIOA时钟
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);	//使能USART时钟
	//USART2_TX   GPIOA.2
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2; //PA.2
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//复用推挽输出
  GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化GPIOA.2
   
  //USART2_RX	  GPIOA.3初始化
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;//PA3
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//浮空输入
  GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化GPIOA.3  
   //USART 初始化设置

	  //UsartNVIC 配置
  NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0 ;//抢占优先级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;		//子优先级
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器
	
	USART_InitStructure.USART_BaudRate = bound;//串口波特率
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式

  USART_Init(USART2, &USART_InitStructure); //初始化串口2
  USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);//开启串口接受中断
  USART_Cmd(USART2, ENABLE);                    //使能串口2 
}

//***************************蓝摇APP通讯*******************//

void LANYAO_APP(int data)//发送数据到蓝摇APP
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
//**************************串口2接收中断***********************//
int Usart2_Receive;
int data_app[10];
u8 flag_mode_app=0;
int anjian_app,huakuai_app,yaogan_app=510;


int USART2_IRQHandler(void)
{	

	if(USART2->SR&(1<<5))//接收到数据
	{
	
		      Usart2_Receive=USART2->DR;
// static u8 i=0;			
//					data_app[i] = USART2->DR;//读取数据		                              
//			switch(i)
//			{
//				case 0:
//					if( data_app[0]==0X79)//读到帧头数据
//					{
//						i++;//下一个字节数据
//					}
//					break;
//				case 1:
//					if( data_app[1]==0X62)//读到帧头数据
//					{
//						i++;//下一个字节数据
//						flag_mode_app=0;//按键数据
//					}
//			   	else	if( data_app[1]==0X76)//读到帧头数据
//					{
//						i++;//下一个字节数据
//						flag_mode_app=1;//滑块数据
//					}
//					else	if( data_app[1]==0X64)//读到帧头数据
//					{
//						i++;//下一个字节数据
//						flag_mode_app=2;//摇杆数据
//					}
//					break;				
//				case 2:
//						i++;//下一个字节数据
//					break;	
//				case 3:            
//            i++;//下一个字节数据	
//					break;
//				case 4:
//					 if(flag_mode_app==0&&data_app[3]==0X0A&&data_app[4]==0X0D)
//						 anjian_app=data_app[2],i=0;//一帧数据完毕
//					 if(flag_mode_app==1&&data_app[3]==0X0A&&data_app[4]==0X0D)
//						 huakuai_app=data_app[2],i=0;//一帧数据完毕
//					 if(flag_mode_app==2)i++;
//					break;
//				case 5:
//					 if(data_app[4]==0X0A&&data_app[5]==0X0D)
//					 yaogan_app= data_app[2]+data_app[3],i=0;//一帧数据完毕				
//					break;
//				default:
//					break;				
//			}
   }
return 0;	

}
//*******************串口3初始化,bound:波特率************************//
void usart3_init(u32 bound)
{  	 
	  //GPIO端口设置
  GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);// 需要使能AFIO时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);	//使能GPIO时钟
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);	//使能USART时钟
//		GPIO_PinRemapConfig(GPIO_PartialRemap_USART3, ENABLE);//引脚重映射
	//USART_TX  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10; //PB10
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//复用推挽输出
  GPIO_Init(GPIOB, &GPIO_InitStructure);   
  //USART_RX	  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;//PB11
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//浮空输入
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  //UsartNVIC 配置
  NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0 ;//抢占优先级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;		//子优先级
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器
   //USART 初始化设置
	USART_InitStructure.USART_BaudRate = bound;//串口波特率
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式
  USART_Init(USART3, &USART_InitStructure);     //初始化串口3
  USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);//开启串口接受中断
  USART_Cmd(USART3, ENABLE);                    //使能串口3 
}

//****************************函数功能：串口3接收中断***************************************//
int Usart3_Receive;
int USART3_IRQHandler(void)
{	

	if(USART3->SR&(1<<5))//接收到数据
	{	      
//		      Usart3_Receive=USART3->DR;
		
static u8 i=0;			
					data_app[i] = USART3->DR;//读取数据		                              
			switch(i)
			{
				case 0:
					if( data_app[0]==0X79)//读到帧头数据
					{
						i++;//下一个字节数据
					}
					break;
				case 1:
					if( data_app[1]==0X62)//读到帧头数据
					{
						i++;//下一个字节数据
						flag_mode_app=0;//按键数据
					}
			   	else	if( data_app[1]==0X76)//读到帧头数据
					{
						i++;//下一个字节数据
						flag_mode_app=1;//滑块数据
					}
					else	if( data_app[1]==0X64)//读到帧头数据
					{
						i++;//下一个字节数据
						flag_mode_app=2;//摇杆数据
					}
					break;				
				case 2:
						i++;//下一个字节数据
					break;	
				case 3:            
            i++;//下一个字节数据	
					break;
				case 4:
					 if(flag_mode_app==0&&data_app[3]==0X0A&&data_app[4]==0X0D)
						 anjian_app=data_app[2],i=0;//一帧数据完毕
					 if(flag_mode_app==1&&data_app[3]==0X0A&&data_app[4]==0X0D)
						 huakuai_app=data_app[2],i=0;//一帧数据完毕
					 if(flag_mode_app==2)i++;
					break;
				case 5:
					 if(data_app[4]==0X0A&&data_app[5]==0X0D)
					 yaogan_app= data_app[2]+data_app[3],i=0;//一帧数据完毕				
					break;
				default:
					break;				
			}
   }
return 0;	
}
