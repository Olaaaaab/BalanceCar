#include "Track.h"
/**************************************************************************
ѭ��ģ��ӿڳ�ʼ��
**************************************************************************/
void Track_Init(void)	
{	
  GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE); //ʹ�ܶ˿�ʱ��
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3|GPIO_Pin_6|GPIO_Pin_7;	  //�˿�����
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;         //��������
  GPIO_Init(GPIOA, &GPIO_InitStructure);					      //�����趨������ʼ��GPIO

}