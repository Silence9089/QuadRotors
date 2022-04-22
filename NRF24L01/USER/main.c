#include "led.h"
#include "delay.h"
#include "sys.h"
#include "usart.h"
#include "lcd.h"
#include "key.h"
#include "spi.h"
#include "24l01.h"   
//ALIENTEK Mini STM32�����巶������24
//����ͨ��ʵ��  
//����֧�֣�www.openedv.com
//������������ӿƼ����޹�˾  

 int main(void)
 { 
	u8 key,mode;
	u16 t=0;			 
	u8 tmp_buf[33]; 
	delay_init();	    	 //��ʱ������ʼ��	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);// �����ж����ȼ�����2
	uart_init(9600);	 	//���ڳ�ʼ��Ϊ9600
	LED_Init();		  		//��ʼ����LED���ӵ�Ӳ���ӿ�
 	KEY_Init();				//������ʼ��
 	NRF24L01_Init();    	//��ʼ��NRF24L01  	  
	 
	 //��ʼ��tmp_buf����
	 for(t=0;t<32;t++)
		{
			tmp_buf[t]='~';	
		}
		tmp_buf[0] = 5;
		tmp_buf[32]=0;//���������		
	 
 	while(NRF24L01_Check())	//���NRF24L01�Ƿ���λ.	
	{
		printf("NRF24L01 Error\r\n");
		delay_ms(200);
	}								   
	printf("NRF24L01 OK\r\n");
	printf("TX MODE:\r\n");
//	printf("RX MODE:\r\n");
	
 	while(1){//�ڸò���ȷ�������ĸ�ģʽ!
		NRF24L01_RX_Mode();		    		    		    				 
			if(NRF24L01_RxPacket(tmp_buf)==0)//һ�����յ���Ϣ,����ʾ����.
			{
				tmp_buf[32]=0;//�����ַ���������
				printf(tmp_buf);    
				
			}
			else {
				printf("Receive nothing\r\n");
				delay_ms(1500);	 
			}
				  
				    

//		NRF24L01_TX_Mode();	   				 
//			if(NRF24L01_TxPacket(tmp_buf)==TX_OK)  //����������
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



























