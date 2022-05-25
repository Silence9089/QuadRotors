#include "stm32f10x.h"
#include "motion_control.h"
#include "delay.h"
#include "pwm.h"
#include "mpu9250.h"
#include "Led.h"
#include "nRF24l01.h"
#include <math.h>


typedef struct{
	float pitch;
	float roll;
	float yaw;
}Euler;

typedef struct{
	float kp;
	float ki;
	float kd;
	Euler err;  //ƫ���ź�
	Euler err_sum;  //ƫ���źŵ��ۻ�
	Euler err_pre;  //��һʱ�̵�ƫ���ź�
	Euler expect;  //����ֵ
	Euler expect_pre;  //��һ�̵����������ڱ�������Ŵ���������ͻ�䣬��ExpectAngleUpdate������
	Euler vs;  //���ٻ���ϵ��
	Euler out;  //���
}Pid;

//�Ƕȣ��⻷���ͽ��ٶȣ��ڻ���
Pid Angle;
Pid Gyro;
Euler init;


//���ض���
short offset_f;  //ƫ��
short offset_b;
short offset_l; 
short offset_r; 

//ң�����ź�
u8 tmp_buf[33];
short remote_base;  //��ֵ 	
short remote_fb;
short remote_lr;
short remote_turn;
u8 remote_mode[4];

//�˲���
#define median_len 71
float median_filter[median_len];  //��ֵ�˲������˵�ң������������š�ȡ������Ϊ�˷���ȡ��λ����
u8 median_i = 0;
#define mean_len 50
float mean_filter_pitch[mean_len];  //�������ھ�ֵ�˲����������������˲�
float mean_filter_roll[mean_len];
float mean_filter_yaw[mean_len];
u8 mean_i = 0;



void Remote_Resolve(void){  //����ң�����ź�
	int i;
	//NRF24L01_RX_Mode();	
	if(NRF24L01_RxPacket(tmp_buf)==0)//һ�����յ���Ϣ,����ʾ����.
	{
		remote_base = (tmp_buf[1]-'0')*1000 + (tmp_buf[2]-'0')*100 + (tmp_buf[3]-'0')*10 + (tmp_buf[4]-'0');
		remote_lr = (tmp_buf[5]-'0')*1000 + (tmp_buf[6]-'0')*100 + (tmp_buf[7]-'0')*10 + (tmp_buf[8]-'0');
		remote_turn = (tmp_buf[9]-'0')*1000 + (tmp_buf[10]-'0')*100 + (tmp_buf[11]-'0')*10 + (tmp_buf[12]-'0');
		remote_fb = (tmp_buf[13]-'0')*1000 + (tmp_buf[14]-'0')*100 + (tmp_buf[15]-'0')*10 + (tmp_buf[16]-'0');
		
		remote_base = (3000 - remote_base) / 2;  //�޷����ĳ�500~1000����pwm��Ӧ
		remote_fb -= 1500;  //��Χ�ĳ�-500~500
		remote_lr -= 1500;
		remote_turn -= 1500;
		
		
		for(i = 0; i < 4; i++)
			remote_mode[i] = tmp_buf[i + 17];
		
//		if(youmen>1500)
//			Led_On();
//		else
//			Led_Off();
//		youmen = (tmp_buf[1]-'0')*1000 + (tmp_buf[2]-'0')*100 + (tmp_buf[3]-'0')*10 + (tmp_buf[4]-'0');
//		
//		tmp_buf[32]=0;//�����ַ���������
//		printf(tmp_buf);    
	}
}

void Motion_Init(void){ //�ñ���΢�ֿ���
	u8 i;
	
	Led_On();
	
	//PID�����������޸ģ���ǰ�Ĳ����ǲο���Դ�ɿ�
	Angle.kp = 2.5;
	Angle.ki = 0;
	Angle.kd = 0;
	Gyro.kp = 0.6;
	Gyro.ki = 0;
	Gyro.kd = 0.01;

	Angle.expect.pitch = 0;
	Angle.expect.roll = 0;
	Angle.expect.yaw = 0;
	Gyro.expect.pitch = 0;
	Gyro.expect.roll = 0;
	Gyro.expect.yaw = 0;
	
	Angle.expect_pre.pitch = 0;
	Angle.expect_pre.roll = 0;
	Angle.expect_pre.yaw = 0;
	Gyro.expect_pre.pitch = 0;
	Gyro.expect_pre.roll = 0;
	Gyro.expect_pre.yaw = 0;	
	
	Angle.err.pitch = 0;
	Angle.err.roll = 0;
	Angle.err.yaw = 0;
	Gyro.err.pitch = 0;
	Gyro.err.roll = 0;
	Gyro.err.yaw = 0;
	
	Angle.err_pre.pitch = 0;
	Angle.err_pre.roll = 0;
	Angle.err_pre.yaw = 0;
	Gyro.err_pre.pitch = 0;
	Gyro.err_pre.roll = 0;
	Gyro.err_pre.yaw = 0;
	
	Angle.err_sum.pitch = 0;  
	Angle.err_sum.roll = 0;
  Angle.err_sum.yaw = 0;
	Gyro.err_sum.pitch = 0;  
	Gyro.err_sum.roll = 0;
  Gyro.err_sum.yaw = 0;
	
	offset_f = 0;  //ƫ��
  offset_b = 0;
  offset_l = 0; 
  offset_r = 0; 
	
//	NRF24L01_RX_Mode();
	
	//���Ž���
	//********************************************
	Remote_Resolve();
	while(remote_base <= 995){  //����10%ռ�ձ�
		Remote_Resolve();	
	}
	TIM_SetCompare1(TIM2,(int)( 1000));
	TIM_SetCompare2(TIM2,(int)( 1000));
	TIM_SetCompare3(TIM2,(int)( 1000));
	TIM_SetCompare4(TIM2,(int)( 1000));
	
	while(remote_base >= 505){  //����5%ռ�ձ�
		Remote_Resolve();
	}
	TIM_SetCompare1(TIM2,(int)( 500));
	TIM_SetCompare2(TIM2,(int)( 500));
	TIM_SetCompare3(TIM2,(int)( 500));
	TIM_SetCompare4(TIM2,(int)( 500));
	
	//********************************************
	
	for(i = 0; i < 70; i++){  
		Update_attitude_Angle();
	}
	for(i = 0; i < 50; i++){  //ȡ���ƽ����Ϊ�ɻ���ʼ��̬����
		Update_attitude_Angle();
		init.pitch += Pitch;
		init.roll += Roll;
		init.yaw += Yaw;
//		mean_filter_pitch[i] = Pitch;
//		mean_filter_roll[i] = Roll;
//		mean_filter_yaw[i] = Yaw;
	}
	
	init.pitch /= 50;
	init.roll /= 50;
	init.yaw /= 50;
	
	Angle.expect.pitch = init.pitch;
	Angle.expect.roll = init.roll;
	Angle.expect.yaw = init.yaw;
	Angle.expect_pre.pitch = init.pitch;
	Angle.expect_pre.roll = init.roll;
	Angle.expect_pre.yaw = init.yaw;
	
	
	Led_Off();
}

float Update_Vs(float err, float lower, float upper){  //���±��ٻ���ϵ��
	float variable_speed;
	
	err = fabs(err);
	if(err > upper || err < lower)
		variable_speed = 0;
	else
		variable_speed = 1.0 - err / upper;
	return variable_speed;
}

float Limit_Offset(float offset){  //��offset�޷�
	if(offset > 150)
		offset = 150;
	else if(offset < -150)
		offset = -150;
	return offset;
}

float Mean_Filter(float* arr, float new_angle){  //�������ھ�ֵ�˲�
	float mean;
	u8 i;
	arr[mean_i] = new_angle;
	for(i = 0; i < mean_len; i++)
	mean += arr[i];
	mean /= mean_len;
	return mean;
}

void AngleControl(void){  //������̬
	Update_attitude_Angle();  //��ȡŷ����
	
//	Pitch = Mean_Filter(mean_filter_pitch, Pitch);
//	Roll = Mean_Filter(mean_filter_roll, Roll);
//	Yaw = Mean_Filter(mean_filter_yaw, Yaw);
	
	if(Angle.expect.yaw - Yaw > 330)
		Angle.expect.yaw -= 360;
	else if(Angle.expect.yaw - Yaw < -330)
		Angle.expect.yaw += 360;
	
	Angle.err_pre = Angle.err;
	
	Angle.err.pitch = Angle.expect.pitch - Pitch;
	Angle.err.roll = Angle.expect.roll - Roll;
	Angle.err.yaw = Angle.expect.yaw - Yaw;
		
	//���±��ٻ���ϵ��
//	vs.pitch = Update_Vs(err.pitch);
//	vs.roll = Update_Vs(err.roll);
//	vs.yaw = Update_Vs(err.yaw);
	
	//����ۼ�
//	err_sum.pitch += vs.pitch * err.pitch;
//	err_sum.roll += vs.roll * err.roll;
//	err_sum.yaw += vs.yaw * err.yaw;

	
	Angle.out.pitch = Angle.kp * Angle.err.pitch + Angle.kd * (Angle.err.pitch - Angle.err_pre.pitch);
	Angle.out.roll = Angle.kp * Angle.err.roll + Angle.kd * (Angle.err.roll - Angle.err_pre.roll);
	Angle.out.yaw = Angle.kp * Angle.err.yaw + Angle.kd * (Angle.err.yaw - Angle.err_pre.yaw);
	
}

void GyroControl(){
	
	Gyro.err_pre = Gyro.err;
	
	//����gyro������˳�����
	Gyro.err.pitch = Gyro.expect.pitch - gyro[0];
	Gyro.err.roll = Gyro.expect.roll - gyro[1];
	Gyro.err.yaw = Gyro.expect.yaw - gyro[2];
	
	
	//����ۼ�
//	err_sum.pitch += vs.pitch * err.pitch;
//	err_sum.roll += vs.roll * err.roll;
//	err_sum.yaw += vs.yaw * err.yaw;

	
	Gyro.out.pitch = Gyro.kp * Gyro.err.pitch + Gyro.kd * (Gyro.err.pitch - Gyro.err_pre.pitch);
	Gyro.out.roll = Gyro.kp * Gyro.err.roll + Gyro.kd * (Gyro.err.roll - Gyro.err_pre.roll);
	Gyro.out.yaw = Gyro.kp * Gyro.err.yaw + Gyro.kd * (Gyro.err.yaw - Gyro.err_pre.yaw);
	
	offset_f = -Gyro.out.pitch - Gyro.out.yaw;
	offset_b = Gyro.out.pitch - Gyro.out.yaw;
	offset_l = -Gyro.out.roll + Gyro.out.yaw;
	offset_r = Gyro.out.roll + Gyro.out.yaw;	
}

short Median(float arr[]){  //���������������λ��
	short i, j, tmp;
	for(i = 0;i < median_len - 1; i++)  //ð������
		for(j = 0; j < median_len - 1 - i; j++)
			if(arr[j] > arr[j + 1]){
				tmp = arr[j];
				arr[j] = arr[j + 1];
				arr[j + 1] = tmp;
			}
	return arr[median_len / 2];
}

void ExpectAngleUpdate(void){
	Remote_Resolve();  //����ң�����ź�
	median_filter[median_i] += remote_turn / 100;
	median_i += 1;
	if(median_i >= median_len){
		Angle.expect.yaw = Median(median_filter);  //��ֵ�˲�
		if (fabs(Angle.expect.yaw - Angle.expect_pre.yaw) > 15)
			Angle.expect.yaw = Angle.expect_pre.yaw;
		Angle.expect_pre.yaw = Angle.expect.yaw;
		median_i = 0;
	}
	Angle.expect.pitch = init.pitch + 10.0 * remote_fb / 500;
	if (fabs(Angle.expect.pitch - Angle.expect_pre.pitch) > 10)
			Angle.expect.pitch = Angle.expect_pre.pitch;
		Angle.expect_pre.pitch = Angle.expect.pitch;
	
	Angle.expect.roll = init.roll + 10.0 * remote_lr / 500;
	if (fabs(Angle.expect.roll- Angle.expect_pre.roll) > 10)
			Angle.expect.roll = Angle.expect_pre.roll;
		Angle.expect_pre.roll = Angle.expect.roll;
}

short Limit_Amplitude(short duty_cycle){  //��ռ�ձ��޷�
	if(duty_cycle > 1000)
		duty_cycle = 1000;
	else if(duty_cycle < 500)
		duty_cycle = 500;
	return duty_cycle;
}

void Motor_F(short duty_cycle){
	duty_cycle = Limit_Amplitude(duty_cycle);
	TIM_SetCompare1(TIM2,(int)( duty_cycle));
}

void Motor_B(short duty_cycle){
	duty_cycle = Limit_Amplitude(duty_cycle);
	TIM_SetCompare2(TIM2,(int)( duty_cycle));
}

void Motor_L(short duty_cycle){
	duty_cycle = Limit_Amplitude(duty_cycle);
	TIM_SetCompare3(TIM2,(int)( duty_cycle));
}

void Motor_R(short duty_cycle){
	duty_cycle = Limit_Amplitude(duty_cycle);
	TIM_SetCompare4(TIM2,(int)( duty_cycle));
}

void Move(void){  //�˶�����
	ExpectAngleUpdate();  //����������̬
	AngleControl();  //������������Ϣ��PID����ƽ��
	GyroControl();
	
	if(remote_mode[1] == '2'){  //����ֹͣ�����ֶ̰����������³�ʼ��
		remote_base = 300;
		offset_f = 0;
		offset_b = 0;
		offset_l = 0;
		offset_r = 0;
		
	}
	
	Motor_F(remote_base + offset_f);
	Motor_B(remote_base + offset_b);
	Motor_L(remote_base + offset_l);
	Motor_R(remote_base + offset_r); 

}