/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdlib.h>
#include ".\motor_control\bsp_motor_control.h"
#include "./led/bsp_led.h"
#include "./key/bsp_key.h" 
#include "./usart/bsp_debug_usart.h"
#include "./Encoder/bsp_encoder.h"
#include "./pid/pid.h"

/* 电机旋转方向 */
__IO int8_t Motor_Direction = 0;
/* 当前时刻总计数值 */
__IO int32_t Capture_Count = 0;
/* 上一时刻总计数值 */
__IO int32_t Last_Count = 0;
/* 电机转轴转速 */
__IO float Shaft_Speed = 0.0f;

__IO float actual_val = 0.0f;

__IO char TaskMuteX = 0;

__IO int Expect_Count = 0;

/**
  * @brief  主函数
  * @param  无
  * @retval 无
  */
int main(void) 
{
  __IO uint16_t ChannelPulse = PWM_MAX_PERIOD_COUNT/2;
  	int i = 1;
//	int PWM = 0;
	float output_val = 0.0f;
	int outputPWM = 0;
	float outputCount = 0.0f;
	float expect_val = 0.0f;
	float expect_shaft_val = 0.0f;


	/* HAL库初始化*/
//  HAL_Init();
	/* 初始化系统时钟为72MHz */
	SystemClock_Config();
	/* 开启复用寄存器时钟 */
	__HAL_RCC_SYSCFG_CLK_ENABLE();
  /* 配置1ms时基为SysTick */
	HAL_InitTick(5);
	/* 初始化按键GPIO */
//	Key_GPIO_Config();
  /* 初始化USART */
	DEBUG_USART_Config();
  
  printf("\r\n——————————野火减速电机编码器测速演示程序——————————\r\n");
  
  /* 电机初始化 */
	motor_init();
  
  /* 设置速度 */
  set_motor_disable();
//	set_motor_speed(ChannelPulse);  
	  set_motor_enable();

  /* 编码器接口初始化 */
	Encoder_Init();
	set_motor_dir(i);
	PID_INIT();
	
	while(1)
	{ 
		if(TaskMuteX == 1)
		{
//			outputCount = PID_POS_CONTORL(Expect_Count, Capture_Count, 1.0, 0.1, 0.02);
			outputCount = PID_POS_CONTORL(Expect_Count, Capture_Count, 2, 0.05, 0);
			expect_shaft_val = outputCount/ENCODER_TOTAL_RESOLUTION*10;
			expect_val = expect_shaft_val/REDUCTION_RATIO;
		
			printf("Capture Count:%d\r\n",Capture_Count);
			printf("output Count:%.4f\r\n",outputCount);
			printf("expect Val:%.4f\r\n",expect_val);
			printf("actual_val：%.4f 转/秒 \r\n", actual_val);/* 输出轴转速 = 转轴转速 / 减速比 */


			output_val = PID_CONTORL(expect_val, actual_val, 150.0, 0.05, 0);

			outputPWM = output_val;
			printf("OUTPUT PWM:%d\r\n\r\n",outputPWM);

			if(outputPWM < 0)
			{
				i = 0;
			}
			else
			{
				i = 1;
			}
			set_motor_dir(i);
			
			if (outputPWM < 0)
			{
				outputPWM = outputPWM * (-1);
			}

						
			if (outputPWM > 1300)
				outputPWM = 1300;
				
			TIM1_SetPWM_pulse(TIM_CHANNEL_2, outputPWM);
			
			TaskMuteX = 0;
		}
		
		
		
//				ChannelPulse = PWM_MAX_PERIOD_COUNT;
//				TIM1_SetPWM_pulse(TIM_CHANNEL_2,PWM);
		

	}
}

/**
  * @brief  SysTick中断回调函数
  * @param  无
  * @retval 无
  */
void HAL_SYSTICK_Callback(void)
{
  static uint16_t i = 0;

  i++;
  if(i == 100)/* 100ms计算一次 */
  {
    /* 电机旋转方向 = 计数器计数方向 */
//    Motor_Direction = __HAL_TIM_IS_TIM_COUNTING_DOWN(&TIM_EncoderHandle);
    
    /* 当前时刻总计数值 = 计数器值 + 计数溢出次数 * ENCODER_TIM_PERIOD  */
    Capture_Count =__HAL_TIM_GET_COUNTER(&TIM_EncoderHandle) + (Encoder_Overflow_Count * ENCODER_TIM_PERIOD);
    
    /* 转轴转速 = 单位时间内的计数值 / 编码器总分辨率 * 时间系数  */
    Shaft_Speed = (float)(Capture_Count - Last_Count) / ENCODER_TOTAL_RESOLUTION * 10 ;

//	  printf("AAAA%d\r\n",Encoder_Overflow_Count);
//    printf("电机方向：%d\r\n", Motor_Direction);
//    printf("单位时间内有效计数值：%d\r\n", Capture_Count - Last_Count);/* 单位时间计数值 = 当前时刻总计数值 - 上一时刻总计数值 */
//    printf("电机转轴处转速：%.2f 转/秒 \r\n", Shaft_Speed);
	actual_val = Shaft_Speed/REDUCTION_RATIO;
//    printf("actual_val：%.2f 转/秒 \r\n", actual_val);/* 输出轴转速 = 转轴转速 / 减速比 */
//    
    /* 记录当前总计数值，供下一时刻计算使用 */
    Last_Count = Capture_Count;
    i = 0;
	TaskMuteX = 1;
  }
}


/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow : 
  *            System Clock source            = PLL (HSE)
  *            SYSCLK(Hz)                     = 72000000
  *            HCLK(Hz)                       = 72000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 2
  *            APB2 Prescaler                 = 1
  *            HSE Frequency(Hz)              = 8000000
  *            HSE PREDIV1                    = 2
  *            PLLMUL                         = 9
  *            Flash Latency(WS)              = 0
  * @param  None
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef clkinitstruct = {0};
  RCC_OscInitTypeDef oscinitstruct = {0};
  
  /* Enable HSE Oscillator and activate PLL with HSE as source */
  oscinitstruct.OscillatorType  = RCC_OSCILLATORTYPE_HSE;
  oscinitstruct.HSEState        = RCC_HSE_ON;
  oscinitstruct.HSEPredivValue  = RCC_HSE_PREDIV_DIV1;
  oscinitstruct.PLL.PLLState    = RCC_PLL_ON;
  oscinitstruct.PLL.PLLSource   = RCC_PLLSOURCE_HSE;
  oscinitstruct.PLL.PLLMUL      = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&oscinitstruct)!= HAL_OK)
  {
    /* Initialization Error */
    while(1); 
  }

  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2 
     clocks dividers */
  clkinitstruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  clkinitstruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  clkinitstruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  clkinitstruct.APB2CLKDivider = RCC_HCLK_DIV1;
  clkinitstruct.APB1CLKDivider = RCC_HCLK_DIV2;  
  if (HAL_RCC_ClockConfig(&clkinitstruct, FLASH_LATENCY_2)!= HAL_OK)
  {
    /* Initialization Error */
    while(1); 
  }
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
