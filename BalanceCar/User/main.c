#include "sys.h"

#define size 5
void Key(void);//*****************************�������
void Touchdown(void);//***********************�������
void APP_Control(void);//*********************APPָ��
void Off_the_ground(void);//******************��ؼ��
void Tracking_module(void);//*****************����ѭ��ģ��
void Ultrasonic_module(void);//***************������ģ��
void Set_Pwm(int motor_A,int motor_B);//******PWM�Ĵ�����ֵ

int  Velocity(int Encoder);//*****************�ٶ�PI������
int  Smooth_Filter(int sample);//*************ƽ���˲�
int  EXTI15_10_IRQHandler(void);//************MPU6050��INT�����ⲿ�жϺ�����MPU6050��INT����ÿ��5�������һ�ε͵�ƽ
int  Balance(float Angle,float Target);//*****ƽ�������
int  Go_straight(float Angle,float Target);//*ֱ�����߸���



float Roll,Pitch,Yaw,gyro_Roll,gyro_Pitch,gyro_Yaw ; //�ǶȲ���

float Angle_error=0;//�����ʵ�ʽǶȵĲ�ֵ
float Yaw_Target;//Yaw�Ƕ�Ŀ��

float Balance_KP=400,Balance_KI=15,Balance_KD=4000;//ƽ����Ƶ�PD����
float	Velocity_KP=300,Velocity_KI=200;//�ٶȿ��Ƶ�PI����
float line_KP=60,line_KD=30;//ֱ�����߸�������PD����

float balance_point=+2.5;//ƽ����㣬���е��أ�����ͨ���۲�ǰ���ͺ��˵��ٶȲ�����ж������ǰ���Ⱥ����ٶȿ죬������
float displacement=0,Velocity_Target=40;//�ٶȿ��Ʋ���

int   Distance;//����������
int   Encoder_A,Encoder_B; //����������  
int   Encoder_Y;//AB�������ں�ֵ
int   Motor_A,Motor_B; //�����PWM����
int   Voltage; //��ص�ѹ 
int   straight_pwm=0,direction_pwm=0,balance_pwm=0;//����PWMֵ

	
u32   count_A,count_B,count_C,count_D;//������
u8    Start_Flag=0,Start_Flag1=1; //������־ 
u8    FLAG_CS,FLAG_A,FLAG_B,FLAG_C,FLAG_D,FLAG_E,FLAG_F,FLAG_G; //��־λ                  
u8    Flag_turn,Flag_Forward,show_flag=0,delay_30,delay_flag;//��־λ
u16   limit; 

int main(void)
{ 
	delay_init();                      //��ʱ��ʼ��
	delay_ms(10);	                     //��ʱ	
	JTAG_Set(JTAG_SWD_DISABLE);        //�ر�JTAG�ӿ�
	JTAG_Set(SWD_ENABLE);              //��SWD�ӿ� �������������SWD�ӿڵ���.		
	delay_ms(10);	                     //��ʱ	
	MY_NVIC_PriorityGroupConfig(2);	   //�����жϷ���		
	usart1_init(115200);               //����1��ʼ��		
	usart3_init(9600);                 //����3��ʼ��	
  delay_ms(50);	                     //��ʱ		
	OLED_Init();                       //OLED��ʼ��
  LED_Init();                        //led��ʼ��
	KEY_Init();                        //������ʼ��	  
	Adc_Init();                        //adc��ʼ��
	Track_Init();                      //ѭ��ģ��ӿڳ�ʼ��
	TIM1_Cap_Init(65535,71);           //������ģ���ʼ��
  delay_ms(50);	                     //��ʱ			
	Motor_PWM_Init(7199,0);            //��ʼ��PWM 10KHZ�������������	
	Encoder_Init_TIM2();               //�������ӿ�	
	Encoder_Init_TIM4();               //�������ӿ�	
	delay_ms(10);	                     //��ʱ	
	IIC_Init();                        //IIC��ʼ��	
  MPU6050_initialize();              //MPU6050��ʼ��	
  DMP_Init();                        //��ʼ��DMP 
	delay_ms(500);                      //��ʱ	
  mpu6050_EXTI_Init();               //MPU6050INT�����жϳ�ʼ��
  delay_ms(10);	                     //��ʱ	
	while(1)
		{		 			
        show_flag=!show_flag;                  //��OLED����λ��
				if(show_flag==0) oled_show();          //oled��ʾ 
        else             
					   mpu6050_send_data(Encoder_A,Motor_A,Roll);//�����������ݵ�������λ��,����1	
			  delay_flag=1;	                         //30ms�жϱ�־λ			  
				while(delay_flag);                     //30ms�ж�
		} 
}



int EXTI15_10_IRQHandler(void) //MPU6050INT����5�����жϺ���
{    
	 if(MPU6050_INT==0)	//��⵽�͵�ƽ
	{     
		EXTI->PR=1<<12; //���EXTI15_10_IRQn�ж����ϵ��жϱ�־λ   

		 
//***************************************��������ȡ***********************************************//		                                          		
		Encoder_A=-Encoder_Read(2);  //ÿ��5�����ȡһ�α�������ֵ 
    Encoder_B=+Encoder_Read(4);  //ÿ��5�����ȡһ�α�������ֵ 
    Read_DMP();//�ǶȽ��ٶ���̬��ȡ
	  Angle_error=Pitch-balance_point;//��ǰ�Ƕ���Ƕ����Ĳ�ֵ
		Distance=Get_distance();//��ȡ����������
		Distance=Smooth_Filter(Distance);//�����������˲�
////***************************************����ָ��***********************************************//			 
		  Key();//�������
		  APP_Control();//APPָ��
			Ultrasonic_module();//������ģ�飬APP����8���볬����ģʽ��
		  Tracking_module();  //����ѭ��ģ�飬���Ϻ���ѭ��ģ����Ĭ�Ϻ���ѭ��ģʽ������Ĭ������APPģʽ

//**************************************С����̬����***************************************************//		
if(Start_Flag==1){	
	                if(Angle_error<+30&&Angle_error>-30)//һ�����ʱ
	  	                   {
													 
			                    Encoder_Y=(Encoder_A+Encoder_B)/2;//������������������ں� 
                          int Pwm_V=0;
                          Pwm_V = + Velocity(Encoder_Y);
//                         if(Pwm_V>+6000)Pwm_V=+6000;
//                         if(Pwm_V<-6000)Pwm_V=-6000;													 
			                    balance_pwm = +Balance(Pitch,balance_point) + Pwm_V;//�ٶ�����������ƽ��ǶȻ��ںϣ������˵����ڵ�ԭ��
			                    if(FLAG_CS==0)//��ѭ��ģʽ 
                               {
		                            if((0==Flag_Forward||1==Flag_Forward||2==Flag_Forward)&&FLAG_A==0&&Flag_turn==0)FLAG_A=1,Yaw_Target=Yaw;//�ǹ���ģʽ������ֱ�����߸�����ͨ���Ƕȵ���
			                          if((3==Flag_Forward||4==Flag_Forward||5==Flag_Forward||6==Flag_Forward||Flag_turn!=0)&&FLAG_A==1)FLAG_A=0,straight_pwm=0; // ����ʱֱ�б�־����������	   
			                          if(FLAG_A==1)straight_pwm = Go_straight(Yaw,Yaw_Target);//ͨ��yaw�Ƕ���PD���ƽ���ֱ�����߸���   
                                if(straight_pwm>+800)straight_pwm=+800;//ֱ�и���PWM�޷�
                                if(straight_pwm<-800)straight_pwm=-800;//ֱ�и���PWM�޷� 	
                               }      
		 	                    Motor_A=balance_pwm - direction_pwm - straight_pwm;//����PWM�ۼ�
			                    Motor_B=balance_pwm + direction_pwm + straight_pwm;//����PWM�ۼ�
													FLAG_G=0;		 
			                   }
			            else FLAG_G++;
									if(FLAG_G>10)Start_Flag=0;//����һ����ǹر�

		             }
//**********************************���ʧ���ж�*************************************//	
    static u16 flag=0;								 
    if(Voltage<1110)flag++;
		else flag=0;						 
		if(flag>800)Start_Flag=0;//����11.1V	�޷�����
	 if(Start_Flag==1)							 
	  {  
		Off_the_ground();//����ʶ��
		Touchdown();//�ŵؼ��
		}        
//************************************PWM������Ƶ��**************************************************//	
	  limit=7100;
    if(Motor_A>+limit)Motor_A=+limit;//PWM�޷���2S��أ�������ߵ�ѹ��ӦTT���
	  if(Motor_A<-limit)Motor_A=-limit;//PWM�޷�
		if(Motor_B>+limit)Motor_B=+limit;//PWM�޷�
	  if(Motor_B<-limit)Motor_B=-limit;//PWM�޷�
		 
		if(Start_Flag==1&&Start_Flag1==1) Set_Pwm(Motor_A,Motor_B);//��ֵ��PWM�Ĵ���  	           
	  else             
		   {
				 Set_Pwm(0,0);
				 Motor_A=0;Motor_B=0;
			   balance_pwm=0;direction_pwm=0;
			   straight_pwm=0;
			   FLAG_A=0;flag=0;
				 FLAG_CS=0;
			   Balance(0,0);Velocity(0);Go_straight(0,0);
			 }//ֹͣʱ���������ֵ
	


//************************************����**************************************************//	
    
//    if(Voltage<1130&&Voltage>600)Led_Flash(20);//��������ʱ������˸
//	  else Led_Flash(100);//led��˸
		count_B+=Get_Adc(2)*3.3*11.0*100/1.0/4096;//��ѹ�����ۼ�
	  count_C++;
		if(count_C==150) Voltage=count_B/150,count_B=0,count_C=0;//��ƽ����ѹ 
		if(delay_flag==1)//��main�����ṩ30ms��ʱ�ж�
		 {
			 if(++delay_30==6)	 delay_30=0,delay_flag=0;
		 }			 
 }
	
	 return 0;	 
} 
/**************************************************************************
�������ܣ�ƽ��Ƕ�PID���� 
**************************************************************************/
int Balance(float Angle,float Target)
{  
   float Bias,PWM;                          
	 static float last_Bias,D_Bias;    	
	 Bias=Angle-Target;                       //�Ƕ�ƫ��
	 D_Bias=Bias - last_Bias;                 //���νǶȲ�ֵ
	 PWM=+Balance_KP*Bias + D_Bias*Balance_KD;//PID�����ۼ�
	 last_Bias=Bias;                          //����Ϊ�ϴ�ƫ��
	 if(Start_Flag==0||Start_Flag1==0)last_Bias=0,D_Bias=0;//ֹͣʱ����������
	 return PWM;
	
}
/**************************************************************************
�������ܣ�ֱ�����ߵ���
**************************************************************************/
int Go_straight(float Angle,float Target)
{  
	static float Last,Bias,Differential;
	       float PWM,Least;
  	Least =Angle-Target;//��ȡƫ��
	  if(Least<+100&&Least>-100) //�ٽ�ת������
		{
          Bias *=0.8;		           //һ�׵�ͨ�˲� 
          Bias += Least*0.2;	     //һ�׵�ͨ�˲� 
	        Differential=Bias-Last;  //��ȡƫ��仯��
	        Last=Bias;               //������һ�ε�ƫ��
		      PWM=Bias*line_KP+Differential*line_KD; //λ�ÿ��� 
		}
		else  {//�ٽ�ת������
			     Bias *=0.8;		         //һ�׵�ͨ�˲� 
           Bias += Least*0.2;	     //һ�׵�ͨ�˲� 
	         Differential=Bias-Last; //��ȡƫ��仯��
	         Last=Bias;              //������һ�ε�ƫ��
		       PWM=-(Bias*line_KP+Differential*line_KD);//λ�ÿ��� 
		      }
    if(Start_Flag==0||Start_Flag1==0)Last=0,Bias=0,Differential=0;//ֹͣʱ����������
	  return PWM;
	
}
/**************************************************************************
�������ܣ��ٶ�PI������
**************************************************************************/
int Velocity(int Encoder)
{  
   static float Velocity_Bias,Velocity_integral;
	        float PWM,Velocity_Least;

		Velocity_Least =Encoder;                                                //�ٶ�ƫ��
		Velocity_Bias *= 0.8;		                                                //һ�׵�ͨ�˲���       
		Velocity_Bias += Velocity_Least*0.2;	                                  //һ�׵�ͨ�˲���    
		Velocity_integral +=Encoder;                                            //���ֳ�λ�� 
		Velocity_integral  =Velocity_integral+displacement;                     //����ǰ������	
			                  if(Velocity_integral>+2000)Velocity_integral=+2000; //�����޷�
	                     	if(Velocity_integral<-2000)Velocity_integral=-2000; //�����޷�			
    PWM = Velocity_Bias*Velocity_KP + Velocity_integral*Velocity_KI/100; //PI�����ۼ�	
    if(Start_Flag==0||Start_Flag1==0)Velocity_Bias=0,Velocity_integral=0;//ֹͣʱ����������
	  return PWM;
}

/**************************************************************************
�������ܣ���ֵ��PWM�Ĵ���
*************************************************************************/
void Set_Pwm(int motor_A,int motor_B)
{
//�Ĵ���
//	  if(motor_A<0)			  CA1=0,CA2=1,PWMA=-(motor_A);//PWM��ֵ��TB6612FNG����оƬ
//  	else	              CA1=1,CA2=0,PWMA=+(motor_A);//PWM��ֵ��TB6612FNG����оƬ
//  	if(motor_B<0)			  CB1=0,CB2=1,PWMB=-(motor_B);//PWM��ֵ��TB6612FNG����оƬ
//  	else	              CB1=1,CB2=0,PWMB=+(motor_B);//PWM��ֵ��TB6612FNG����оƬ
//�Ĵ���	
//	  if(motor_A<0)   INA=0,PWMA=-motor_A;//PWM��ֵ��L298N ��BTS7960 ��A4950   
//	  else            INA=1,PWMA=7199-motor_A;//PWM��ֵ��L298N ��BTS7960 ��A4950	����7199�ǵ��PWM���Զ���װ�ؼĴ��������ֵ�������õ��PWMƵ�ʵ�ʱ�����õġ�
//	  if(motor_B<0)   INB=0,PWMB=-motor_B;//PWM��ֵ��L298N ��BTS7960 ��A4950
//	  else            INB=1,PWMB=7199-motor_B;//PWM��ֵ��L298N ��BTS7960 ��A4950  ����7199�ǵ��PWM���Զ���װ�ؼĴ��������ֵ�������õ��PWMƵ�ʵ�ʱ�����õġ�
	
//�⺯��	
	  if(motor_A<0)			  CA1=0,CA2=1,TIM_SetCompare3(TIM3,-motor_A);//PWM��ֵTIM3_CH3��TB6612FNG����оƬ ������ͨIO�ں�һ��PWM��
  	else	              CA1=1,CA2=0,TIM_SetCompare3(TIM3,+motor_A);//PWM��ֵTIM3_CH3��TB6612FNG����оƬ
  	if(motor_B<0)			  CB1=0,CB2=1,TIM_SetCompare4(TIM3,-motor_B);//PWM��ֵTIM3_CH4��TB6612FNG����оƬ
  	else	              CB1=1,CB2=0,TIM_SetCompare4(TIM3,+motor_B);//PWM��ֵTIM3_CH4��TB6612FNG����оƬ
//�⺯��
//		if(motor_A<0)     INA=0,TIM_SetCompare3(TIM3,-motor_A); //PWM��ֵTIM3_CH3��L298N ��BTS7960 ��A4950 ���� ��һ����ͨIO�ں�һ��PWN�ڵĿ�������
//	  else              INA=1,TIM_SetCompare3(TIM3,7199-motor_A);// ����7199�ǵ��PWM���Զ���װ�ؼĴ��������ֵ�������õ��PWMƵ�ʵ�ʱ�����õġ�
//		if(motor_B<0)     INB=0,TIM_SetCompare4(TIM3,-motor_B); //PWM��ֵTIM3_CH4��L298N ��BTS7960 ��A4950 ���� ��һ����ͨIO�ں�һ��PWN�ڵĿ�������
//	  else              INB=1,TIM_SetCompare4(TIM3,7199-motor_B);// 	����7199�ǵ��PWM���Զ���װ�ؼĴ��������ֵ�������õ��PWMƵ�ʵ�ʱ�����õġ�
}

/**************************************************************************
�������ܣ�ƽ���˲�
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
�������ܣ�����
**************************************************************************/
void Key(void)
{	
static u8 flag_1=0;
	
flag_1=click_N_Double(70);	
if(flag_1==1&&Start_Flag==0)Start_Flag=1,flag_1=0,Start_Flag1=1;//��������
if(flag_1==1&&Start_Flag==1)Start_Flag=0,flag_1=0,Start_Flag1=0;//�����ر�

}                                                                    
/**************************************************************************
�������ܣ�����ʧ��
**************************************************************************/
void Off_the_ground(void)
{	
	static int count_1=0,count_2=0,flag_2=0,flag_3=0;
	static float angle_1,angle_2;
	if(Start_Flag1==1){

                                  if(balance_pwm>=+limit){ //���������ڻ�����޷�ֵʱ��ʼ�ж�                        
																		                                   count_1 +=Encoder_A;//���������ֳ�λ��
		                                                                   count_2 +=Encoder_B;//���������ֳ�λ��
																	                                     flag_3=0;
																	                                     if(flag_2==0)angle_1=Pitch;//��ȡ��ǰ�Ƕ�
																	                                     flag_2++;
																	                                    }
																	//�÷�����PWMʱ���ǶȲ�����仯�������λ�Ƴ���400���ж����
		                              if(flag_2>10&&(Pitch-angle_1)>-0.5&&count_1>+200&&count_2>+200)Start_Flag1=0,count_1=0,count_2=0,flag_2=0;
																  if(flag_2>10)count_1=0,count_2=0,flag_2=0;
																	
																  if(balance_pwm<=-limit){//���������ڻ�����޷�ֵʱ��ʼ�ж�         
																		                                   count_1 +=Encoder_A;//���������ֳ�λ��
		                                                                   count_2 +=Encoder_B;//���������ֳ�λ��   
																	                                     flag_2=0;
																	                                     if(flag_3==0)angle_2=Pitch;//��ȡ��ǰ�Ƕ�
																	                                     flag_3++;
																	                                    }
																	//�÷�����PWMʱ���ǶȲ�����仯����С����λ�Ƴ���400���ж����
		                              if(flag_3>10&&(Pitch-angle_2)<+0.5&&count_1<-200&&count_2<-200)Start_Flag1=0,count_1=0,count_2=0,flag_3=0;
																	if(flag_3>10)count_1=0,count_2=0,flag_3=0;
																	
																	if(Encoder_A>+3||Encoder_B>+3)flag_3=0;
																	if(Encoder_A<-3||Encoder_B<-3)flag_2=0;
	                   }
}
/**************************************************************************
�������ܣ��ŵ�ʹ��
**************************************************************************/
void Touchdown(void)
{	
static int count_1=0;
 if(Start_Flag1==0){
	                 count_1++; 
	                 if(count_1>300)
										 {
	                    if(Angle_error>-5&&Angle_error<+5)
									    if(Encoder_A!=0||Encoder_B!=0)Start_Flag1=1,count_1=0; //ƽ����㸽����������΢ת�����ж��ŵ�
									    }
                   }//����㸽��
 
}
/**************************************************************************
APP����ң��ָ��
**************************************************************************/
void  APP_Control(void)
{
if(FLAG_CS==0)
{
		 if(anjian_app==1)Start_Flag=1,anjian_app=0;//����1����
	   if(anjian_app==2)Start_Flag=0,anjian_app=0;//����2�ر�
	   if(anjian_app==3)Velocity_Target=200;//����
	   if(anjian_app==4||anjian_app==8)Velocity_Target=100;//����
          if(yaogan_app>=248&&yaogan_app<=292)Flag_Forward=1,Flag_turn=0;//ҡ���ϲ�ǰ��
     else if(yaogan_app>=68&&yaogan_app<=112) Flag_Forward=2,Flag_turn=0;//ҡ���²�����
	   else if(yaogan_app>=203&&yaogan_app<=247)Flag_Forward=3,Flag_turn=0;//ҡ��������ǰת
     else if(yaogan_app>=293&&yaogan_app<=337)Flag_Forward=4,Flag_turn=0;//ҡ��������ǰת
	   else if(yaogan_app>=113&&yaogan_app<=157)Flag_Forward=5,Flag_turn=0;//ҡ���������ת
	   else if(yaogan_app>=23&&yaogan_app<=67)  Flag_Forward=6,Flag_turn=0;//ҡ�������Һ�ת
     else Flag_Forward=0;

	        if(yaogan_app>=158&&yaogan_app<=202)Flag_Forward=0,Flag_turn=1;//ҡ��������ת
     else if((yaogan_app>=0&&yaogan_app<=22)||(yaogan_app>=338&&yaogan_app<=360))Flag_Forward=0,Flag_turn=2;//ҡ���Ҳ�����ת
     else Flag_turn=0;
	
 	   if(yaogan_app==510)Flag_Forward=0,Flag_turn=0;
	
				 if(1==Flag_Forward&&Flag_turn==0)    	  displacement=+Velocity_Target,direction_pwm  =0;//ǰ��
		else if(2==Flag_Forward&&Flag_turn==0)	      displacement=-Velocity_Target,direction_pwm  =0;//����
	  else if(3==Flag_Forward&&Flag_turn==0)	      { displacement=+Velocity_Target; direction_pwm -=20; if(direction_pwm<-200)direction_pwm=-200;}//��ת��
	  else if(4==Flag_Forward&&Flag_turn==0)	      { displacement=+Velocity_Target; direction_pwm +=20; if(direction_pwm>+200)direction_pwm=+200;}//��ת��
		else if(5==Flag_Forward&&Flag_turn==0)	      { displacement=-Velocity_Target; direction_pwm +=20; if(direction_pwm>+200)direction_pwm=+200;}//��ת��
	  else if(6==Flag_Forward&&Flag_turn==0)	      { displacement=-Velocity_Target; direction_pwm -=20; if(direction_pwm<-200)direction_pwm=-200;}//��ת��
    else displacement=0;
			
				  if(1==Flag_turn&&0==Flag_Forward)    	  {direction_pwm -=10;if(direction_pwm<-3000)direction_pwm=-3000;}	      //����ת
	   else if(2==Flag_turn&&0==Flag_Forward)	      {direction_pwm +=10;if(direction_pwm>+3000)direction_pwm=+3000;}        //����ת
     if(0==Flag_turn&&0==Flag_Forward)direction_pwm=0;
 }		
}
/**************************************************************************
�������ܣ�������ģ��Ӧ��
**************************************************************************/
void Ultrasonic_module(void)
{	
 static u8 flag_1=0,count_1=0;
 static float KP=12,KI=4000,Last,Bias,Differential,Least;
if(FLAG_CS==0)//��ѭ��ģʽ���ɽ��볬����Ӧ��
{                   
if(anjian_app==8){//APP����8���볬���������Լ�����ģʽ
	           if(Distance>10&&0==Flag_turn&&0==Flag_Forward)//������볬����ģ��
		                  { 
		                    if(Distance<300)//����ģʽ����ǰ����ϰ��ﱣ��100�ľ���	,ͨ��PI����λ�ÿ���		                                
			                    {
  	                       Least =Distance-150;     //��ȡƫ��
                           Bias *=0.8;		          //һ�׵�ͨ�˲�
                           Bias += Least*0.2;	      //һ�׵�ͨ�˲�
	                         Differential=Bias-Last;  //��ȡƫ��仯��
	                         Last=Bias;               //������һ�ε�ƫ��
		                       displacement=Bias*KP + Differential*KI; //λ�ÿ��� 
			                     if(displacement<-30)displacement=-30;   //����ģʽ�ٶ��޷�
				                   if(displacement>+30)displacement=+30;   //����ģʽ�ٶ��޷�												
			                    }
			                  else displacement=0;
		                   }	
											
              if(Distance>10&&Flag_turn==0&&Flag_Forward==1)	//ǰ��ʱ���ǰ�����ϰ�������ֹͣ��Ȼ����ת�ܿ������λ����ǰ��
		                {
					                if(Distance<400&&flag_1==0)count_1++;
										 else if(Distance>400&&flag_1==0)count_1=0;
											
											if(count_1>20)flag_1=1,count_1=0;
					           if(flag_1==1){Yaw_Target=Yaw_Target-90;if(Yaw_Target<-179)Yaw_Target=+179-(-179-Yaw_Target);flag_1=2;}//�ı�yaw�Ƕ�Ŀ��ֵ����ת����ֱ�и�������PWM����
					           if((Yaw-Yaw_Target)<+5&&(Yaw-Yaw_Target)>-5)flag_1=0;
										 	if(displacement<-15)displacement=-15;//����ģʽ�ٶ��޷�
				              if(displacement>+15)displacement=+15;//����ģʽ�ٶ��޷�
		                }
              if(Flag_turn==0&&Flag_Forward==0)direction_pwm=0;
			          }
}
}
/**************************************************************************
�������ܣ�����ѭ��ģ��
**************************************************************************/
void Tracking_module(void)
{	
   static float KI=300,Last,Differential,Least;	
	 static float median=0,rate_s;
	//����ʹ����һ����·���⣬���Զȵͣ���ѭ��Ч������Ӱ��
	//ѭ��ģ����ð�װ�ڳ���ǰ�ˣ�����ת����Ҫ��ǰ�������ڿ����Ͽ��Եõ���ǰ����Ϣ
	//�������룬�ߵ�ƽ������
	//��⵽��ɫ����ɫ���͵�ƽ0������
	//��⵽��ɫ���ߵ�ƽ1������
	//�ǰ�ɫʱ������������ɫ�����ȫ�𣬸ߵ�ƽ
  //��ģ������շ������Ϊ23mm���Ⱥ�ɫ�߶�Ҫ������ζ�źܴ����ͬһʱ��ֻ����һ�������ں�������
	//�����߶��ں��ʸ߶�

 if(TRACK_1==1&&TRACK_2==1&&TRACK_3==1)FLAG_CS=0;//�ж�δ����ѭ��ģ��
 else{//����ѭ��ģ��
	        FLAG_CS=1;//�ж�����ѭ��ģ��

				  displacement=+10;//ѭ��ʱǰ�����ٶ��޷������ı��Ӱ��ѭ��Ч��
	 
           if(TRACK_1==0&&TRACK_2==1&&TRACK_3==0)rate_s=0,median=0;//��--��--�ף����㣬��ֹ�����ϴ��ڵ����������
      else if(TRACK_1==1&&TRACK_2==1&&TRACK_3==0)rate_s=+2;//��--��--��
      else if(TRACK_1==1&&TRACK_2==0&&TRACK_3==0)rate_s=+6;//��--��--��
      else if(TRACK_1==0&&TRACK_2==1&&TRACK_3==1)rate_s=-2;//��--��--��
      else if(TRACK_1==0&&TRACK_2==0&&TRACK_3==1)rate_s=-6;//��--��--��
      else if(TRACK_1==0&&TRACK_2==0&&TRACK_3==0)rate_s=0;//��--��--�ף�����Ϊ��������һ��ֵ��

	                         median +=rate_s;//ͨ���ۻ��ķ�ʽ��ģ����������
	 
					  	             Least =median-0;     //��ȡƫ��									 
	                         Differential =Least-Last;  //��ȡƫ��仯��
	                         Last=Least;               //������һ�ε�ƫ��	 
		                       direction_pwm=-(Differential*KI); //ֻ��һ��������ԭ����ÿ��ֻתһ���̶�ֵ���бƿ��ƣ�
													
		                       if(Start_Flag==0||Start_Flag1==0)rate_s=0,median=0,Last=0,Differential=0,displacement=0;				
				
     }
 
}
/*****************end******************/
