#include "sys.h"

#define size 5
void Key(void);//*****************************按键检测
void Touchdown(void);//***********************触摸检测
void APP_Control(void);//*********************APP指令
void Off_the_ground(void);//******************离地检测
void Tracking_module(void);//*****************红外循迹模块
void Ultrasonic_module(void);//***************超声波模块
void Set_Pwm(int motor_A,int motor_B);//******PWM寄存器赋值

int  Velocity(int Encoder);//*****************速度PI控制器
int  Smooth_Filter(int sample);//*************平滑滤波
int  EXTI15_10_IRQHandler(void);//************MPU6050的INT引脚外部中断函数，MPU6050的INT引脚每隔5毫秒会跳一次低电平
int  Balance(float Angle,float Target);//*****平衡控制器
int  Go_straight(float Angle,float Target);//*直行行走辅助



float Roll,Pitch,Yaw,gyro_Roll,gyro_Pitch,gyro_Yaw ; //角度参数

float Angle_error=0;//零点与实际角度的差值
float Yaw_Target;//Yaw角度目标

float Balance_KP=400,Balance_KI=15,Balance_KD=4000;//平衡控制的PD参数
float	Velocity_KP=300,Velocity_KI=200;//速度控制的PI参数
float line_KP=60,line_KD=30;//直线行走辅助控制PD参数

float balance_point=+2.5;//平衡零点，与机械相关，可以通过观察前进和后退的速度差距来判定，如果前进比后退速度快，就增加
float displacement=0,Velocity_Target=40;//速度控制参数

int   Distance;//超声波距离
int   Encoder_A,Encoder_B; //编码器读数  
int   Encoder_Y;//AB编码器融合值
int   Motor_A,Motor_B; //电机的PWM参数
int   Voltage; //电池电压 
int   straight_pwm=0,direction_pwm=0,balance_pwm=0;//各环PWM值

	
u32   count_A,count_B,count_C,count_D;//计数器
u8    Start_Flag=0,Start_Flag1=1; //启动标志 
u8    FLAG_CS,FLAG_A,FLAG_B,FLAG_C,FLAG_D,FLAG_E,FLAG_F,FLAG_G; //标志位                  
u8    Flag_turn,Flag_Forward,show_flag=0,delay_30,delay_flag;//标志位
u16   limit; 

int main(void)
{ 
	delay_init();                      //延时初始化
	delay_ms(10);	                     //延时	
	JTAG_Set(JTAG_SWD_DISABLE);        //关闭JTAG接口
	JTAG_Set(SWD_ENABLE);              //打开SWD接口 可以利用主板的SWD接口调试.		
	delay_ms(10);	                     //延时	
	MY_NVIC_PriorityGroupConfig(2);	   //设置中断分组		
	usart1_init(115200);               //串口1初始化		
	usart3_init(9600);                 //串口3初始化	
  delay_ms(50);	                     //延时		
	OLED_Init();                       //OLED初始化
  LED_Init();                        //led初始化
	KEY_Init();                        //按键初始化	  
	Adc_Init();                        //adc初始化
	Track_Init();                      //循迹模块接口初始化
	TIM1_Cap_Init(65535,71);           //超声波模块初始化
  delay_ms(50);	                     //延时			
	Motor_PWM_Init(7199,0);            //初始化PWM 10KHZ，用于驱动电机	
	Encoder_Init_TIM2();               //编码器接口	
	Encoder_Init_TIM4();               //编码器接口	
	delay_ms(10);	                     //延时	
	IIC_Init();                        //IIC初始化	
  MPU6050_initialize();              //MPU6050初始化	
  DMP_Init();                        //初始化DMP 
	delay_ms(500);                      //延时	
  mpu6050_EXTI_Init();               //MPU6050INT引脚中断初始化
  delay_ms(10);	                     //延时	
	while(1)
		{		 			
        show_flag=!show_flag;                  //错开OLED与上位机
				if(show_flag==0) oled_show();          //oled显示 
        else             
					   mpu6050_send_data(Encoder_A,Motor_A,Roll);//发送三个数据到匿名上位机,串口1	
			  delay_flag=1;	                         //30ms中断标志位			  
				while(delay_flag);                     //30ms中断
		} 
}



int EXTI15_10_IRQHandler(void) //MPU6050INT引脚5毫秒中断函数
{    
	 if(MPU6050_INT==0)	//检测到低电平
	{     
		EXTI->PR=1<<12; //清除EXTI15_10_IRQn中断线上的中断标志位   

		 
//***************************************传感器获取***********************************************//		                                          		
		Encoder_A=-Encoder_Read(2);  //每隔5毫秒读取一次编码器的值 
    Encoder_B=+Encoder_Read(4);  //每隔5毫秒读取一次编码器的值 
    Read_DMP();//角度角速度姿态获取
	  Angle_error=Pitch-balance_point;//当前角度与角度零点的差值
		Distance=Get_distance();//获取超声波距离
		Distance=Smooth_Filter(Distance);//超声波距离滤波
////***************************************动作指令***********************************************//			 
		  Key();//按键检测
		  APP_Control();//APP指令
			Ultrasonic_module();//超声波模块，APP按键8进入超声波模式，
		  Tracking_module();  //红外循迹模块，接上红外循迹模块则默认红外循迹模式，否则默认蓝牙APP模式

//**************************************小车姿态控制***************************************************//		
if(Start_Flag==1){	
	                if(Angle_error<+30&&Angle_error>-30)//一定倾角时
	  	                   {
													 
			                    Encoder_Y=(Encoder_A+Encoder_B)/2;//两个电机编码器数据融合 
                          int Pwm_V=0;
                          Pwm_V = + Velocity(Encoder_Y);
//                         if(Pwm_V>+6000)Pwm_V=+6000;
//                         if(Pwm_V<-6000)Pwm_V=-6000;													 
			                    balance_pwm = +Balance(Pitch,balance_point) + Pwm_V;//速度正反馈环和平衡角度环融合，利用了倒立摆的原理
			                    if(FLAG_CS==0)//非循迹模式 
                               {
		                            if((0==Flag_Forward||1==Flag_Forward||2==Flag_Forward)&&FLAG_A==0&&Flag_turn==0)FLAG_A=1,Yaw_Target=Yaw;//非拐弯模式，启动直线行走辅助，通过角度调节
			                          if((3==Flag_Forward||4==Flag_Forward||5==Flag_Forward||6==Flag_Forward||Flag_turn!=0)&&FLAG_A==1)FLAG_A=0,straight_pwm=0; // 拐弯时直行标志和数据清零	   
			                          if(FLAG_A==1)straight_pwm = Go_straight(Yaw,Yaw_Target);//通过yaw角度作PD控制进行直线行走辅助   
                                if(straight_pwm>+800)straight_pwm=+800;//直行辅助PWM限幅
                                if(straight_pwm<-800)straight_pwm=-800;//直行辅助PWM限幅 	
                               }      
		 	                    Motor_A=balance_pwm - direction_pwm - straight_pwm;//各环PWM累加
			                    Motor_B=balance_pwm + direction_pwm + straight_pwm;//各环PWM累加
													FLAG_G=0;		 
			                   }
			            else FLAG_G++;
									if(FLAG_G>10)Start_Flag=0;//超过一定倾角关闭

		             }
//**********************************电机失能判断*************************************//	
    static u16 flag=0;								 
    if(Voltage<1110)flag++;
		else flag=0;						 
		if(flag>800)Start_Flag=0;//低于11.1V	无法启动
	 if(Start_Flag==1)							 
	  {  
		Off_the_ground();//提起识别
		Touchdown();//着地检测
		}        
//************************************PWM输出控制电机**************************************************//	
	  limit=7100;
    if(Motor_A>+limit)Motor_A=+limit;//PWM限幅，2S电池，限制最高电压适应TT马达
	  if(Motor_A<-limit)Motor_A=-limit;//PWM限幅
		if(Motor_B>+limit)Motor_B=+limit;//PWM限幅
	  if(Motor_B<-limit)Motor_B=-limit;//PWM限幅
		 
		if(Start_Flag==1&&Start_Flag1==1) Set_Pwm(Motor_A,Motor_B);//赋值给PWM寄存器  	           
	  else             
		   {
				 Set_Pwm(0,0);
				 Motor_A=0;Motor_B=0;
			   balance_pwm=0;direction_pwm=0;
			   straight_pwm=0;
			   FLAG_A=0;flag=0;
				 FLAG_CS=0;
			   Balance(0,0);Velocity(0);Go_straight(0,0);
			 }//停止时清除各参数值
	


//************************************其它**************************************************//	
    
//    if(Voltage<1130&&Voltage>600)Led_Flash(20);//电量不足时快速闪烁
//	  else Led_Flash(100);//led闪烁
		count_B+=Get_Adc(2)*3.3*11.0*100/1.0/4096;//电压采样累计
	  count_C++;
		if(count_C==150) Voltage=count_B/150,count_B=0,count_C=0;//求平均电压 
		if(delay_flag==1)//给main函数提供30ms延时判断
		 {
			 if(++delay_30==6)	 delay_30=0,delay_flag=0;
		 }			 
 }
	
	 return 0;	 
} 
/**************************************************************************
函数功能：平衡角度PID控制 
**************************************************************************/
int Balance(float Angle,float Target)
{  
   float Bias,PWM;                          
	 static float last_Bias,D_Bias;    	
	 Bias=Angle-Target;                       //角度偏差
	 D_Bias=Bias - last_Bias;                 //两次角度差值
	 PWM=+Balance_KP*Bias + D_Bias*Balance_KD;//PID控制累加
	 last_Bias=Bias;                          //保存为上次偏差
	 if(Start_Flag==0||Start_Flag1==0)last_Bias=0,D_Bias=0;//停止时各参数清零
	 return PWM;
	
}
/**************************************************************************
函数功能：直线行走调整
**************************************************************************/
int Go_straight(float Angle,float Target)
{  
	static float Last,Bias,Differential;
	       float PWM,Least;
  	Least =Angle-Target;//获取偏差
	  if(Least<+100&&Least>-100) //临界转换方向
		{
          Bias *=0.8;		           //一阶低通滤波 
          Bias += Least*0.2;	     //一阶低通滤波 
	        Differential=Bias-Last;  //获取偏差变化率
	        Last=Bias;               //保存上一次的偏差
		      PWM=Bias*line_KP+Differential*line_KD; //位置控制 
		}
		else  {//临界转换方向
			     Bias *=0.8;		         //一阶低通滤波 
           Bias += Least*0.2;	     //一阶低通滤波 
	         Differential=Bias-Last; //获取偏差变化率
	         Last=Bias;              //保存上一次的偏差
		       PWM=-(Bias*line_KP+Differential*line_KD);//位置控制 
		      }
    if(Start_Flag==0||Start_Flag1==0)Last=0,Bias=0,Differential=0;//停止时各参数清零
	  return PWM;
	
}
/**************************************************************************
函数功能：速度PI控制器
**************************************************************************/
int Velocity(int Encoder)
{  
   static float Velocity_Bias,Velocity_integral;
	        float PWM,Velocity_Least;

		Velocity_Least =Encoder;                                                //速度偏差
		Velocity_Bias *= 0.8;		                                                //一阶低通滤波器       
		Velocity_Bias += Velocity_Least*0.2;	                                  //一阶低通滤波器    
		Velocity_integral +=Encoder;                                            //积分出位移 
		Velocity_integral  =Velocity_integral+displacement;                     //控制前进后退	
			                  if(Velocity_integral>+2000)Velocity_integral=+2000; //积分限幅
	                     	if(Velocity_integral<-2000)Velocity_integral=-2000; //积分限幅			
    PWM = Velocity_Bias*Velocity_KP + Velocity_integral*Velocity_KI/100; //PI控制累加	
    if(Start_Flag==0||Start_Flag1==0)Velocity_Bias=0,Velocity_integral=0;//停止时各参数清零
	  return PWM;
}

/**************************************************************************
函数功能：赋值给PWM寄存器
*************************************************************************/
void Set_Pwm(int motor_A,int motor_B)
{
//寄存器
//	  if(motor_A<0)			  CA1=0,CA2=1,PWMA=-(motor_A);//PWM赋值，TB6612FNG驱动芯片
//  	else	              CA1=1,CA2=0,PWMA=+(motor_A);//PWM赋值，TB6612FNG驱动芯片
//  	if(motor_B<0)			  CB1=0,CB2=1,PWMB=-(motor_B);//PWM赋值，TB6612FNG驱动芯片
//  	else	              CB1=1,CB2=0,PWMB=+(motor_B);//PWM赋值，TB6612FNG驱动芯片
//寄存器	
//	  if(motor_A<0)   INA=0,PWMA=-motor_A;//PWM赋值，L298N 或BTS7960 或A4950   
//	  else            INA=1,PWMA=7199-motor_A;//PWM赋值，L298N 或BTS7960 或A4950	数字7199是电机PWM的自动重装载寄存器的最大值，在设置电机PWM频率的时候设置的。
//	  if(motor_B<0)   INB=0,PWMB=-motor_B;//PWM赋值，L298N 或BTS7960 或A4950
//	  else            INB=1,PWMB=7199-motor_B;//PWM赋值，L298N 或BTS7960 或A4950  数字7199是电机PWM的自动重装载寄存器的最大值，在设置电机PWM频率的时候设置的。
	
//库函数	
	  if(motor_A<0)			  CA1=0,CA2=1,TIM_SetCompare3(TIM3,-motor_A);//PWM赋值TIM3_CH3，TB6612FNG驱动芯片 两个普通IO口和一个PWM口
  	else	              CA1=1,CA2=0,TIM_SetCompare3(TIM3,+motor_A);//PWM赋值TIM3_CH3，TB6612FNG驱动芯片
  	if(motor_B<0)			  CB1=0,CB2=1,TIM_SetCompare4(TIM3,-motor_B);//PWM赋值TIM3_CH4，TB6612FNG驱动芯片
  	else	              CB1=1,CB2=0,TIM_SetCompare4(TIM3,+motor_B);//PWM赋值TIM3_CH4，TB6612FNG驱动芯片
//库函数
//		if(motor_A<0)     INA=0,TIM_SetCompare3(TIM3,-motor_A); //PWM赋值TIM3_CH3，L298N 或BTS7960 或A4950 驱动 用一个普通IO口和一个PWN口的控制例程
//	  else              INA=1,TIM_SetCompare3(TIM3,7199-motor_A);// 数字7199是电机PWM的自动重装载寄存器的最大值，在设置电机PWM频率的时候设置的。
//		if(motor_B<0)     INB=0,TIM_SetCompare4(TIM3,-motor_B); //PWM赋值TIM3_CH4，L298N 或BTS7960 或A4950 驱动 用一个普通IO口和一个PWN口的控制例程
//	  else              INB=1,TIM_SetCompare4(TIM3,7199-motor_B);// 	数字7199是电机PWM的自动重装载寄存器的最大值，在设置电机PWM频率的时候设置的。
}

/**************************************************************************
函数功能：平滑滤波
**************************************************************************/
int Smooth_Filter(int sample)
{	
  int i,Filter_value= 0; 
  static  int array[size+1]={0},Sum = 0;
       	    
   array[size] = sample;
              for(i = 0; i < size; i++) 
               {
                array[i] = array[i + 1];	  
                Sum  += array[i];
               }
                Filter_value = (int)(Sum / size);
							  Sum = 0;
      return Filter_value;
           
}
/**************************************************************************
函数功能：按键
**************************************************************************/
void Key(void)
{	
static u8 flag_1=0;
	
flag_1=click_N_Double(70);	
if(flag_1==1&&Start_Flag==0)Start_Flag=1,flag_1=0,Start_Flag1=1;//单击启动
if(flag_1==1&&Start_Flag==1)Start_Flag=0,flag_1=0,Start_Flag1=0;//单击关闭

}                                                                    
/**************************************************************************
函数功能：提起失能
**************************************************************************/
void Off_the_ground(void)
{	
	static int count_1=0,count_2=0,flag_2=0,flag_3=0;
	static float angle_1,angle_2;
	if(Start_Flag1==1){

                                  if(balance_pwm>=+limit){ //正反馈大于或等于限幅值时开始判定                        
																		                                   count_1 +=Encoder_A;//编码器积分出位移
		                                                                   count_2 +=Encoder_B;//编码器积分出位移
																	                                     flag_3=0;
																	                                     if(flag_2==0)angle_1=Pitch;//获取当前角度
																	                                     flag_2++;
																	                                    }
																	//该方向满PWM时，角度不满足变化量变大并且位移超过400则判定离地
		                              if(flag_2>10&&(Pitch-angle_1)>-0.5&&count_1>+200&&count_2>+200)Start_Flag1=0,count_1=0,count_2=0,flag_2=0;
																  if(flag_2>10)count_1=0,count_2=0,flag_2=0;
																	
																  if(balance_pwm<=-limit){//正反馈大于或等于限幅值时开始判定         
																		                                   count_1 +=Encoder_A;//编码器积分出位移
		                                                                   count_2 +=Encoder_B;//编码器积分出位移   
																	                                     flag_2=0;
																	                                     if(flag_3==0)angle_2=Pitch;//获取当前角度
																	                                     flag_3++;
																	                                    }
																	//该方向满PWM时，角度不满足变化量减小并且位移超过400则判定离地
		                              if(flag_3>10&&(Pitch-angle_2)<+0.5&&count_1<-200&&count_2<-200)Start_Flag1=0,count_1=0,count_2=0,flag_3=0;
																	if(flag_3>10)count_1=0,count_2=0,flag_3=0;
																	
																	if(Encoder_A>+3||Encoder_B>+3)flag_3=0;
																	if(Encoder_A<-3||Encoder_B<-3)flag_2=0;
	                   }
}
/**************************************************************************
函数功能：着地使能
**************************************************************************/
void Touchdown(void)
{	
static int count_1=0;
 if(Start_Flag1==0){
	                 count_1++; 
	                 if(count_1>300)
										 {
	                    if(Angle_error>-5&&Angle_error<+5)
									    if(Encoder_A!=0||Encoder_B!=0)Start_Flag1=1,count_1=0; //平衡零点附近，轮子稍微转动即判定着地
									    }
                   }//在零点附近
 
}
/**************************************************************************
APP蓝牙遥控指令
**************************************************************************/
void  APP_Control(void)
{
if(FLAG_CS==0)
{
		 if(anjian_app==1)Start_Flag=1,anjian_app=0;//按键1启动
	   if(anjian_app==2)Start_Flag=0,anjian_app=0;//按键2关闭
	   if(anjian_app==3)Velocity_Target=200;//高速
	   if(anjian_app==4||anjian_app==8)Velocity_Target=100;//低速
          if(yaogan_app>=248&&yaogan_app<=292)Flag_Forward=1,Flag_turn=0;//摇杆上拨前进
     else if(yaogan_app>=68&&yaogan_app<=112) Flag_Forward=2,Flag_turn=0;//摇杆下拨后退
	   else if(yaogan_app>=203&&yaogan_app<=247)Flag_Forward=3,Flag_turn=0;//摇杆左上左前转
     else if(yaogan_app>=293&&yaogan_app<=337)Flag_Forward=4,Flag_turn=0;//摇杆右上右前转
	   else if(yaogan_app>=113&&yaogan_app<=157)Flag_Forward=5,Flag_turn=0;//摇杆左下左后转
	   else if(yaogan_app>=23&&yaogan_app<=67)  Flag_Forward=6,Flag_turn=0;//摇杆右下右后转
     else Flag_Forward=0;

	        if(yaogan_app>=158&&yaogan_app<=202)Flag_Forward=0,Flag_turn=1;//摇杆左拨左自转
     else if((yaogan_app>=0&&yaogan_app<=22)||(yaogan_app>=338&&yaogan_app<=360))Flag_Forward=0,Flag_turn=2;//摇杆右拨右自转
     else Flag_turn=0;
	
 	   if(yaogan_app==510)Flag_Forward=0,Flag_turn=0;
	
				 if(1==Flag_Forward&&Flag_turn==0)    	  displacement=+Velocity_Target,direction_pwm  =0;//前进
		else if(2==Flag_Forward&&Flag_turn==0)	      displacement=-Velocity_Target,direction_pwm  =0;//后退
	  else if(3==Flag_Forward&&Flag_turn==0)	      { displacement=+Velocity_Target; direction_pwm -=20; if(direction_pwm<-200)direction_pwm=-200;}//左转弯
	  else if(4==Flag_Forward&&Flag_turn==0)	      { displacement=+Velocity_Target; direction_pwm +=20; if(direction_pwm>+200)direction_pwm=+200;}//右转弯
		else if(5==Flag_Forward&&Flag_turn==0)	      { displacement=-Velocity_Target; direction_pwm +=20; if(direction_pwm>+200)direction_pwm=+200;}//左转弯
	  else if(6==Flag_Forward&&Flag_turn==0)	      { displacement=-Velocity_Target; direction_pwm -=20; if(direction_pwm<-200)direction_pwm=-200;}//右转弯
    else displacement=0;
			
				  if(1==Flag_turn&&0==Flag_Forward)    	  {direction_pwm -=10;if(direction_pwm<-3000)direction_pwm=-3000;}	      //左自转
	   else if(2==Flag_turn&&0==Flag_Forward)	      {direction_pwm +=10;if(direction_pwm>+3000)direction_pwm=+3000;}        //右自转
     if(0==Flag_turn&&0==Flag_Forward)direction_pwm=0;
 }		
}
/**************************************************************************
函数功能：超声波模块应用
**************************************************************************/
void Ultrasonic_module(void)
{	
 static u8 flag_1=0,count_1=0;
 static float KP=12,KI=4000,Last,Bias,Differential,Least;
if(FLAG_CS==0)//非循迹模式方可进入超声波应用
{                   
if(anjian_app==8){//APP按键8进入超声波避障以及跟随模式
	           if(Distance>10&&0==Flag_turn&&0==Flag_Forward)//如果接入超声波模块
		                  { 
		                    if(Distance<300)//跟随模式，与前面的障碍物保持100的距离	,通过PI进行位置控制		                                
			                    {
  	                       Least =Distance-150;     //获取偏差
                           Bias *=0.8;		          //一阶低通滤波
                           Bias += Least*0.2;	      //一阶低通滤波
	                         Differential=Bias-Last;  //获取偏差变化率
	                         Last=Bias;               //保存上一次的偏差
		                       displacement=Bias*KP + Differential*KI; //位置控制 
			                     if(displacement<-30)displacement=-30;   //跟随模式速度限幅
				                   if(displacement>+30)displacement=+30;   //跟随模式速度限幅												
			                    }
			                  else displacement=0;
		                   }	
											
              if(Distance>10&&Flag_turn==0&&Flag_Forward==1)	//前进时如果前方有障碍，减速停止，然后右转避开后方向回位继续前进
		                {
					                if(Distance<400&&flag_1==0)count_1++;
										 else if(Distance>400&&flag_1==0)count_1=0;
											
											if(count_1>20)flag_1=1,count_1=0;
					           if(flag_1==1){Yaw_Target=Yaw_Target-90;if(Yaw_Target<-179)Yaw_Target=+179-(-179-Yaw_Target);flag_1=2;}//改变yaw角度目标值进行转向，与直行辅助共用PWM参数
					           if((Yaw-Yaw_Target)<+5&&(Yaw-Yaw_Target)>-5)flag_1=0;
										 	if(displacement<-15)displacement=-15;//避障模式速度限幅
				              if(displacement>+15)displacement=+15;//避障模式速度限幅
		                }
              if(Flag_turn==0&&Flag_Forward==0)direction_pwm=0;
			          }
}
}
/**************************************************************************
函数功能：红外循迹模块
**************************************************************************/
void Tracking_module(void)
{	
   static float KI=300,Last,Differential,Least;	
	 static float median=0,rate_s;
	//这里使用了一种三路红外，线性度低，对循迹效果略有影响
	//循迹模块最好安装在车子前端，比旋转中心要提前，这样在控制上可以得到超前的信息
	//超出距离，高电平，灯灭
	//检测到白色，黄色，低电平0，灯亮
	//检测到黑色，高电平1，灯灭
	//非白色时，三个灯在无色差地面全灭，高电平
  //该模块红外收发器间距为23mm，比黑色线都要宽，这意味着很大可能同一时间只能有一个灯是在黑线上面
	//离地面高度在合适高度

 if(TRACK_1==1&&TRACK_2==1&&TRACK_3==1)FLAG_CS=0;//判定未连接循迹模块
 else{//接入循迹模块
	        FLAG_CS=1;//判定连接循迹模块

				  displacement=+10;//循迹时前进的速度限幅，若改变会影响循迹效果
	 
           if(TRACK_1==0&&TRACK_2==1&&TRACK_3==0)rate_s=0,median=0;//白--黑--白（清零，防止理论上存在的数据溢出）
      else if(TRACK_1==1&&TRACK_2==1&&TRACK_3==0)rate_s=+2;//黑--黑--白
      else if(TRACK_1==1&&TRACK_2==0&&TRACK_3==0)rate_s=+6;//黑--白--白
      else if(TRACK_1==0&&TRACK_2==1&&TRACK_3==1)rate_s=-2;//白--黑--黑
      else if(TRACK_1==0&&TRACK_2==0&&TRACK_3==1)rate_s=-6;//白--白--黑
      else if(TRACK_1==0&&TRACK_2==0&&TRACK_3==0)rate_s=0;//白--白--白（无作为，保持上一数值）

	                         median +=rate_s;//通过累积的方式，模拟线性增减
	 
					  	             Least =median-0;     //获取偏差									 
	                         Differential =Least-Last;  //获取偏差变化率
	                         Last=Least;               //保存上一次的偏差	 
		                       direction_pwm=-(Differential*KI); //只有一个参数，原理是每次只转一个固定值（夹逼控制）
													
		                       if(Start_Flag==0||Start_Flag1==0)rate_s=0,median=0,Last=0,Differential=0,displacement=0;				
				
     }
 
}
/*****************end******************/
