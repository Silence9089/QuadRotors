#include "delay.h"
#include "sys.h"
#include "usart.h"
#include "spi.h"
#include "nRF24l01.h"   
#include "mpu9250.h"
#include "stm32f10x.h"


 int main(void)
 { 
	 //���еĳ�ʼ��
	 //**************************************************************
	u16 t=0;			 
	u8 tmp_buf[33];  //nrf���ͺͽ��յ���Ϣ�������������
	delay_init();	    	 //��ʱ������ʼ��	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);// �����ж����ȼ�����2
	uart_init(9600);	 	//���ڳ�ʼ��Ϊ9600
 	NRF24L01_Init();    	//��ʼ��NRF24L01  	  
	Mpu9250_Init();
	 
	 //��ʼ��tmp_buf����
	 for(t=0;t<32;t++)
		{
			tmp_buf[t]='*';	
		}
		tmp_buf[0] = 5;
		tmp_buf[32]=0;//���������		
	 
		/*���NRF24L01�Ƿ���λ�����û��nrf�Ļ���ע����
			��Ȼ��һֱ�ڼ���ѭ�������ȥ*/	
// 	while(NRF24L01_Check())	
//	{
//		printf("NRF24L01 Error\r\n");
//		delay_ms(200);
//	}								   
//	printf("NRF24L01 OK\r\n");
		
	//**************************************************************
	
		
		
		/*tips: ��usart.c���ض���printf��
		����ʹ��printf�ǽ�����ͨ�����ڷ��ͳ�ȥ��������ԡ�
		�����͵�����Ϊ�ַ���������ʹ��u8�ͣ���tmp_buf���Ǹ����ͣ���*/
 	while(1){
		//nrf�Ľ���ģʽ��ʹ�õĻ���Ҫ���·��ķ���ģʽע�͵�
		//**************************************************************
//		NRF24L01_RX_Mode();	
//		printf("RX MODE:\r\n");		
//			if(NRF24L01_RxPacket(tmp_buf)==0)//һ�����յ���Ϣ,����ʾ����.
//			{
//				tmp_buf[32]=0;//�����ַ���������
//				printf(tmp_buf);    
//				
//			}
//			else {
//				printf("Receive nothing\r\n");
//				delay_ms(1500);	 
//			}
		//***************************************************************
				    

			//nrf�ķ���ģʽ��ʹ�õĻ���Ҫ���Ϸ��Ľ���ģʽע�͵�
		//**************************************************************
//		NRF24L01_TX_Mode();	
//			printf("TX MODE:\r\n");			
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
		//**************************************************************
	} 
}



























