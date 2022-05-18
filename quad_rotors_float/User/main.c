/*
飞行方式：十字型
接线方式：PA0-PA3分别对应前后左右螺旋桨
陀螺仪布置方式：x轴朝前，z轴朝上

上电后板子灯亮，此时正在初始化，灯灭则表示初始化结束。

遥控器信号说明：
0：保留位，无作用
1-4：油门
5-8：左右平移
9-12：左右旋转
13-16：前后平移
17-20：模式控制（17右手长按；18右手短按；19左手长按；20左手短按）

右手短按：紧急停止
*/

#include "delay.h"
#include "sys.h"
#include "usart.h"
#include "spi.h"
#include "nRF24l01.h"   
#include "mpu9250.h"
#include "stm32f10x.h"
#include "pwm.h"
#include "motor.h"
#include "Led.h"
#include "stm32_iic.h"
#include "motion_control.h"


 int main(void)
 { 
	 //所有的初始化
	 //**************************************************************
	u8 tmp_buf[33];  //nrf发送和接收的信息都在这个数组里
	Led_Init();
	Led_On();
	 
	delay_init();	    	 //延时函数初始化	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);// 设置中断优先级分组2
	uart_init(9600);	 	//串口初始化为9600
 	NRF24L01_Init();    	//初始化NRF24L01  
	i2cInit();      //IIC总线的初始化
	delay_ms(10);
	Mpu_Init(1);
	//motor_Init();
	TIM_PWM_Init(9999,143);//不分频。PWM频率=72000/(9999+1)/(143+1)=50Hz
	 
		while(NRF24L01_Check())	
	{
		printf("NRF24L01 Error\r\n");
		delay_ms(200);
	}								   
	printf("NRF24L01 OK\r\n");
	NRF24L01_RX_Mode();
	
	Motion_Init();
	 
	Led_Off();
	 
	 //初始化tmp_buf数组
//	 for(t=0;t<32;t++)
//		{
//			tmp_buf[t]='*';	
//		}
//		tmp_buf[0] = 5;
//		tmp_buf[32]=0;//加入结束符		
	 
		/*检查NRF24L01是否在位，如果没连nrf的话就注掉，
			不然会一直在检测的循环里出不去*/	

		
	//**************************************************************
	

		/*tips: 在usart.c中重定义printf，
		现在使用printf是将内容通过串口发送出去，方便调试。
		但发送的内容为字符串，建议使用u8型（即tmp_buf的那个类型）。*/
		
 	while(1){
		
		//nrf的接收模式，使用的话需要将下方的发送模式注释掉
		//**************************************************************
		//NRF24L01_RX_Mode();	
//		printf(":RX MODE\r\n");		
//			if(NRF24L01_RxPacket(tmp_buf)==0)//一旦接收到信息,则显示出来.
//			{
//				if(youmen>1500)
//					Led_On();
//				else
//					Led_Off();
//				youmen = (tmp_buf[1]-'0')*1000 + (tmp_buf[2]-'0')*100 + (tmp_buf[3]-'0')*10 + (tmp_buf[4]-'0');
//				
//				tmp_buf[32]=0;//加入字符串结束符
//				printf(tmp_buf);    
//				
//			}
//			else {
//				printf("Receive nothing\r\n");
//				delay_ms(1500);	 
//			}
		//***************************************************************
			
			
			
		//nrf的发送模式，使用的话需要将上方的接收模式注释掉
		//**************************************************************
//		NRF24L01_TX_Mode();	
//			printf("TX MODE:\r\n");			
//			if(NRF24L01_TxPacket(tmp_buf)==TX_OK)  //如果发送完成
//			{
//				printf(tmp_buf);
//				printf("Send Succeeded\r\n");  
//				   
//			}else
//			{								
// 				printf("Send Failed "); 
//			};
//			delay_ms(1500);				 
	
		//**************************************************************
			
			
		//PWM测试
		//****************************************************************
//		TIM_SetCompare1(TIM2,(int)(10000-(- youmen / 2 + 1500)));
//		TIM_SetCompare2(TIM2,(int)(10000-(- youmen / 2 + 1500)));
//		TIM_SetCompare3(TIM2,(int)(10000-(- youmen / 2 + 1500)));
//		TIM_SetCompare4(TIM2,(int)(10000-(- youmen / 2 + 1500)));
		//***************************************************************
		
		//陀螺仪测试
		//***************************************************************
//			Update_attitude_Angle();

//			printf("Pitch:");
//			printf("%f ",Pitch);

//			printf("Roll:");
//			printf("%f ",Roll);     

//			printf("Yaw:");        
//			printf("%f ",Yaw);

//			Update_Magnetometer();
//			printf("M:%d \n",Direction);
		//***************************************************************
		
		//运动控制测试
		//***************************************************************
		Move();
		//***************************************************************
	}
}
