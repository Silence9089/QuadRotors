#ifndef __LED_H
#define __LED_H	 
#include "sys.h"
//////////////////////////////////////////////////////////////////////////////////	 
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//ALIENTEKս��STM32������
//LED��������	   
//11111111@ALIENTEK
//������̳:www.11111111.com
//�޸�����:2012/9/2
//�汾��V1.0
//��Ȩ���У�����ؾ���
//Copyright(C)111111111111111111111111111 2009-2019
//All rights reserved									  
////////////////////////////////////////////////////////////////////////////////// 
#define MOTOR1 PAout(0)	// PA0
#define MOTOR2 PDout(1)	// PA1	
#define MOTOR3 PDout(2)	// PA2	
#define MOTOR4 PDout(3)	// PA3	

void motor_Init(void);//��ʼ��

		 				    
#endif
