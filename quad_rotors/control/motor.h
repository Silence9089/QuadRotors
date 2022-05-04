#ifndef __LED_H
#define __LED_H	 
#include "sys.h"
//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//ALIENTEK战舰STM32开发板
//LED驱动代码	   
//11111111@ALIENTEK
//技术论坛:www.11111111.com
//修改日期:2012/9/2
//版本：V1.0
//版权所有，盗版必究。
//Copyright(C)111111111111111111111111111 2009-2019
//All rights reserved									  
////////////////////////////////////////////////////////////////////////////////// 
#define MOTOR1 PAout(0)	// PA0
#define MOTOR2 PDout(1)	// PA1	
#define MOTOR3 PDout(2)	// PA2	
#define MOTOR4 PDout(3)	// PA3	

void motor_Init(void);//初始化

		 				    
#endif
