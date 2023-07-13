#ifndef __USART_H
#define __USART_H
#include "sys.h"
void usart1_send(u8 data);
void usart1_send_char(u8 c);
void usart1_init(u32 bound);
void usart1_niming_report(u8 fun,u8*data,u8 len);
void mpu6050_send_data(float pitch_,float roll_,float yaw_);
void usart2_send(u8 data);
void usart2_init(u32 bound);
void usart3_init(u32 bound);
void LANYAO_APP(int data);
#endif	   

















