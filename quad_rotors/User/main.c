#include "delay.h"
#include "sys.h"
#include "usart.h"
#include "spi.h"
#include "nRF24l01.h"   
#include "mpu9250.h"
#include "stm32f10x.h"


 int main(void)
 { 
	 //所有的初始化
	 //**************************************************************
	u16 t=0;			 
	u8 tmp_buf[33];  //nrf发送和接收的信息都在这个数组里
	delay_init();	    	 //延时函数初始化	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);// 设置中断优先级分组2
	uart_init(9600);	 	//串口初始化为9600
 	NRF24L01_Init();    	//初始化NRF24L01  	  
	Mpu9250_Init();
	 
	 //初始化tmp_buf数组
	 for(t=0;t<32;t++)
		{
			tmp_buf[t]='*';	
		}
		tmp_buf[0] = 5;
		tmp_buf[32]=0;//加入结束符		
	 
		/*检查NRF24L01是否在位，如果没连nrf的话就注掉，
			不然会一直在检测的循环里出不去*/	
// 	while(NRF24L01_Check())	
//	{
//		printf("NRF24L01 Error\r\n");
//		delay_ms(200);
//	}								   
//	printf("NRF24L01 OK\r\n");
		
	//**************************************************************
	
		
		
		/*tips: 在usart.c中重定义printf，
		现在使用printf是将内容通过串口发送出去，方便调试。
		但发送的内容为字符串，建议使用u8型（即tmp_buf的那个类型）。*/
 	while(1){
		//nrf的接收模式，使用的话需要将下方的发送模式注释掉
		//**************************************************************
//		NRF24L01_RX_Mode();	
//		printf("RX MODE:\r\n");		
//			if(NRF24L01_RxPacket(tmp_buf)==0)//一旦接收到信息,则显示出来.
//			{
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
//	
		//**************************************************************
	} 
}



























