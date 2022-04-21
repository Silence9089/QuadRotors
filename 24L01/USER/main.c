#include "led.h"
#include "delay.h"
#include "sys.h"
#include "usart.h"
#include "lcd.h"
#include "key.h"
#include "spi.h"
#include "24l01.h"   
//ALIENTEK Mini STM32开发板范例代码24
//无线通信实验  
//技术支持：www.openedv.com
//广州市星翼电子科技有限公司  

 int main(void)
 { 
	u8 key,mode;
	u16 t=0;			 
	u8 tmp_buf[33]; 
	delay_init();	    	 //延时函数初始化	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);// 设置中断优先级分组2
	uart_init(9600);	 	//串口初始化为9600
	LED_Init();		  		//初始化与LED连接的硬件接口
 	KEY_Init();				//按键初始化
 	NRF24L01_Init();    	//初始化NRF24L01  	  
	 
	 //初始化tmp_buf数组
	 for(t=0;t<32;t++)
		{
			tmp_buf[t]='~';	
		}
		tmp_buf[0] = 5;
		tmp_buf[32]=0;//加入结束符		
	 
 	while(NRF24L01_Check())	//检查NRF24L01是否在位.	
	{
		printf("NRF24L01 Error\r\n");
		delay_ms(200);
	}								   
	printf("NRF24L01 OK\r\n");
	printf("TX MODE:\r\n");
//	printf("RX MODE:\r\n");
	
 	while(1){//在该部分确定进入哪个模式!
		NRF24L01_RX_Mode();		    		    		    				 
			if(NRF24L01_RxPacket(tmp_buf)==0)//一旦接收到信息,则显示出来.
			{
				tmp_buf[32]=0;//加入字符串结束符
				printf(tmp_buf);    
				
			}
			else {
				printf("Receive nothing\r\n");
				delay_ms(1500);	 
			}
				  
				    

//		NRF24L01_TX_Mode();	   				 
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
	} 
}



























