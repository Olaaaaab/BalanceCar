#include ".\motor_control\bsp_motor_control.h"
#include "./usart/bsp_debug_usart.h"
#include <math.h>
#include <stdlib.h>

static motor_dir_t direction  = MOTOR_FWD;     // ��¼����
static uint16_t    dutyfactor = 0;             // ��¼ռ�ձ�

static void sd_gpio_config(void)
{
  GPIO_InitTypeDef GPIO_InitStruct;
  
  /* ��ʱ��ͨ���������Ŷ˿�ʱ��ʹ�� */
	SHUTDOWN_GPIO_CLK_ENABLE();
  
  /* ����IO��ʼ�� */
	/*�����������*/
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	/*������������ */ 
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	/*ѡ��Ҫ���Ƶ�GPIO����*/	
	GPIO_InitStruct.Pin = SHUTDOWN_PIN;
  
	/*���ÿ⺯����ʹ���������õ�GPIO_InitStructure��ʼ��GPIO*/
  HAL_GPIO_Init(SHUTDOWN_GPIO_PORT, &GPIO_InitStruct);
}

/**
  * @brief  �����ʼ��
  * @param  ��
  * @retval ��
  */
void motor_init(void)
{
  TIMx_Configuration();     // ��ʼ����� 1
  sd_gpio_config();
}

/**
  * @brief  ���õ���ٶ�
  * @param  v: �ٶȣ�ռ�ձȣ�
  * @retval ��
  */
void set_motor_speed(uint16_t v)
{
  dutyfactor = v;
  
  if (direction == MOTOR_FWD)
  {
    SET_FWD_COMPAER(dutyfactor);     // �����ٶ�
  }
  else
  {
    SET_REV_COMPAER(dutyfactor);     // �����ٶ�
  }
}

/**
  * @brief  ���õ������
  * @param  ��
  * @retval ��
  */
//void set_motor_direction(motor_dir_t dir)
//{
//  direction = dir;
//  
//  SET_FWD_COMPAER(0);     // �����ٶ�Ϊ 0
//  SET_REV_COMPAER(0);     // �����ٶ�Ϊ 0
//  
//  HAL_Delay(200);         // ��ʱһ��
//  
//  if (direction == MOTOR_FWD)
//  {
//    SET_FWD_COMPAER(dutyfactor);     // �����ٶ�
//    SET_REV_COMPAER(0);              // �����ٶ�
//  }
//  else
//  {
//    SET_FWD_COMPAER(0);              // �����ٶ�
//    SET_REV_COMPAER(dutyfactor);     // �����ٶ�
//  }
//}

void set_motor_dir(int dir)
{
	if(dir == 1)
	{
		HAL_GPIO_WritePin(DIR_GPIO_PORT, DIR_PIN_0, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(DIR_GPIO_PORT, DIR_PIN_1, GPIO_PIN_SET);
	}
	if(dir == 0)
	{
		HAL_GPIO_WritePin(DIR_GPIO_PORT, DIR_PIN_0, GPIO_PIN_SET);
		HAL_GPIO_WritePin(DIR_GPIO_PORT, DIR_PIN_1, GPIO_PIN_RESET);
	}
}


/**
  * @brief  ʹ�ܵ��
  * @param  ��
  * @retval ��
  */
//void set_motor_enable(void)
//{
//  MOTOR_ENABLE_SD();
//  MOTOR_FWD_ENABLE();
//  MOTOR_REV_ENABLE();
//}
void set_motor_enable(void)
{
  MOTOR_ENABLE_SD();
	
  GPIO_InitTypeDef GPIO_InitStruct;
  /* ��ʱ��ͨ���������Ŷ˿�ʱ��ʹ�� */
	DIR_GPIO_CLK_ENABLE();
  
  /* ����IO��ʼ�� */
	/*�����������*/
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	/*������������ */ 
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	/*ѡ��Ҫ���Ƶ�GPIO����*/	
	GPIO_InitStruct.Pin = DIR_PIN_0;
  
	/*���ÿ⺯����ʹ���������õ�GPIO_InitStructure��ʼ��GPIO*/
  HAL_GPIO_Init(DIR_GPIO_PORT, &GPIO_InitStruct);
	
	GPIO_InitStruct.Pin = DIR_PIN_1;
  HAL_GPIO_Init(DIR_GPIO_PORT, &GPIO_InitStruct);

	
}



/**
  * @brief  ���õ��
  * @param  ��
  * @retval ��
  */
void set_motor_disable(void)
{
  MOTOR_DISABLE_SD();
//  MOTOR_FWD_DISABLE();
//  MOTOR_REV_DISABLE();
}

/**
  * @brief  ��ӡ��������
  * @param  ��
  * @retval ��
  */
void show_help(void)
{
    printf("����������������������������Ұ��ֱ�����ٵ��������ʾ���򡪡�������������������������\n\r");
    printf("��������(�Իس�����)��\n\r");
    printf("< ? >       -�����˵�\n\r");
    printf("v [data]     -���õ�����ٶȣ���Χ��0��%d��\n\r", PWM_MAX_PERIOD_COUNT);
    printf("d [data]     -���õ���ķ���%d:����ת��%d:����ת\n\r", MOTOR_FWD, MOTOR_REV);
		printf("����ʾ��:d 0,ע��d��0�京һ���ո�,����ʱĩβ���л��з����ڴ��������й�ѡ��������\n\r");
}

/**
  * @brief  �����ڽ��յ�������
  * @param  ��
  * @retval ��
  */
void deal_serial_data(void)
{
    static char showflag =1;
    int dec_temp=0;
    int speed_temp=0;
    
    //���յ���ȷ��ָ���Ϊ1
    char okCmd = 0;
  
    if (showflag)
    {
      show_help();
      showflag = !showflag;
    }

    //����Ƿ���յ�ָ��
    if(receive_cmd == 1)
    {
      if(UART_RxBuffer[0] == 'v' || UART_RxBuffer[0] == 'V')
      {
        //�����ٶ�
        if(UART_RxBuffer[1] == ' ')
        {
          speed_temp = atoi((char const *)UART_RxBuffer+2);
          if(speed_temp>=0 && speed_temp <= PWM_MAX_PERIOD_COUNT)
          {
            set_motor_speed(speed_temp);
            printf("\n\r�ٶ�: %d\n\r", speed_temp);
            okCmd = 1;
          }
        }
      }
      else if(UART_RxBuffer[0] == 'd')
      {
        //���÷���
        if(UART_RxBuffer[1] == ' ')
        {
          dec_temp = atoi((char const *)UART_RxBuffer+2);

          if(dec_temp>=0)
          {
            set_motor_dir(dec_temp);
            printf("\n\r����:%s\n\r", dec_temp ? "����ת" : "����ת");
            okCmd = 1;
          }
        }
      }
      else if(UART_RxBuffer[0] == '?')
      {
        //��ӡ��������
        show_help();
        okCmd = 1;
      }
      //���ָ���������ӡ��������
      if(okCmd != 1)
      {
        printf("\n\r ������������������...\n\r");
        show_help();
      }

      //��մ��ڽ��ջ�������
      receive_cmd = 0;
      uart_FlushRxBuffer();

    }
}

/*********************************************END OF FILE**********************/

