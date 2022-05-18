#include "stm32f10x.h"
#include "motion_control.h"
#include "delay.h"
#include "pwm.h"
#include "mpu9250.h"
#include "Led.h"
#include "nRF24l01.h"
#include <math.h>

//位置式pid控制的一些参数
float kp = 20;
float ki = 0;
float kd = 10;

typedef struct{
	float pitch;
	float roll;
	float yaw;
}Euler;


Euler err;  //偏差信号
Euler err_sum;
Euler err_pre;  //上一时刻的偏差信号
Euler init;  //上电时的欧拉角（也即飞机水平时的欧拉角）
Euler expect;  //期望的欧拉角
Euler vs;  //变速积分系数
Euler uk;  //控制信号

//被控对象
short offset_f;  //偏移
short offset_b;
short offset_l; 
short offset_r; 

//遥控器信号
u8 tmp_buf[33];
short remote_base;  //基值 	
short remote_fb;
short remote_lr;
short remote_turn;
u8 remote_mode[4];

//滤波器
#define median_len 71
float median_filter[median_len];  //中值滤波器，滤掉遥控器的脉冲干扰。取奇数是为了方便取中位数。
u8 median_i = 0;
#define mean_len 50
float mean_filter_pitch[mean_len];  //滑动窗口均值滤波，对陀螺仪数据滤波
float mean_filter_roll[mean_len];
float mean_filter_yaw[mean_len];
u8 mean_i = 0;



void Remote_Resolve(void){  //解算遥控器信号
	int i;
	//NRF24L01_RX_Mode();	
	if(NRF24L01_RxPacket(tmp_buf)==0)//一旦接收到信息,则显示出来.
	{
		remote_base = (tmp_buf[1]-'0')*1000 + (tmp_buf[2]-'0')*100 + (tmp_buf[3]-'0')*10 + (tmp_buf[4]-'0');
		remote_lr = (tmp_buf[5]-'0')*1000 + (tmp_buf[6]-'0')*100 + (tmp_buf[7]-'0')*10 + (tmp_buf[8]-'0');
		remote_turn = (tmp_buf[9]-'0')*1000 + (tmp_buf[10]-'0')*100 + (tmp_buf[11]-'0')*10 + (tmp_buf[12]-'0');
		remote_fb = (tmp_buf[13]-'0')*1000 + (tmp_buf[14]-'0')*100 + (tmp_buf[15]-'0')*10 + (tmp_buf[16]-'0');
		
		remote_base = (3000 - remote_base) / 2;  //限幅，改成500~1000，跟pwm对应
		remote_fb -= 1500;  //范围改成-500~500
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
//		tmp_buf[32]=0;//加入字符串结束符
//		printf(tmp_buf);    
	}
}

void Motion_Init(void){ //用比例微分控制
	u8 i;
	
	Led_On();
	
	uk.pitch = 0;
  uk.roll = 0;
	uk.yaw = 0;
	
	err.pitch = 0;
	err.roll = 0;
	err.yaw = 0;
	
//	err_sum.pitch = 0;  
//	err_sum.roll = 0;
//  err_sum.yaw = 0;
	
	err_pre.pitch = 0;
	err_pre.roll = 0;
	err_pre.yaw = 0;
	
	vs.pitch = 0;
	vs.roll = 0;
	vs.yaw = 0;
	
	offset_f = 0;  //偏移
  offset_b = 0;
  offset_l = 0; 
  offset_r = 0; 
	
//	NRF24L01_RX_Mode();
	
	//油门解锁
	//********************************************
	Remote_Resolve();
	while(remote_base >= 995){  //发送10%占空比
		Remote_Resolve();
	}
	TIM_SetCompare1(TIM2,(int)(10000 - 1000));
	TIM_SetCompare2(TIM2,(int)(10000 - 1000));
	TIM_SetCompare3(TIM2,(int)(10000 - 1000));
	TIM_SetCompare4(TIM2,(int)(10000 - 1000));
	
	while(remote_base <= 505){  //发送5%占空比
		Remote_Resolve();
	}
	TIM_SetCompare1(TIM2,(int)(10000 - 500));
	TIM_SetCompare2(TIM2,(int)(10000 - 500));
	TIM_SetCompare3(TIM2,(int)(10000 - 500));
	TIM_SetCompare4(TIM2,(int)(10000 - 500));
	
	//********************************************
	
	for(i = 0; i < 70; i++){  
		Update_attitude_Angle();
	}
	for(i = 0; i < mean_len; i++){  //取多次平均作为飞机初始姿态数据
		Update_attitude_Angle();
		init.pitch += Pitch;
		init.roll += Roll;
		init.yaw += Yaw;
		mean_filter_pitch[i] = Pitch;
		mean_filter_roll[i] = Roll;
		mean_filter_yaw[i] = Yaw;
	}
	
	init.pitch /= 50;
	init.roll /= 50;
	init.yaw /= 50;
	
	expect.pitch = init.pitch;
	expect.roll = init.roll;
	expect.yaw = init.yaw;
	
	Led_Off();
}

float Update_Vs(float err_angle){  //更新变速积分系数
	float variable_speed;
	float lower_angle = 1;
	float upper_angle = 10;
	
	err_angle = fabs(err_angle);
	if(err_angle > upper_angle || err_angle < lower_angle)
		variable_speed = 0;
	else
		variable_speed = 1.0 - err_angle / upper_angle;
	return variable_speed;
}

float Limit_Offset(float offset){  //对offset限幅
	if(offset > 150)
		offset = 150;
	else if(offset < -150)
		offset = -150;
	return offset;
}

float Mean_Filter(float* arr, float new_angle){  //滑动窗口均值滤波
	float mean;
	u8 i;
	arr[mean_i] = new_angle;
	for(i = 0; i < mean_len; i++)
	mean += arr[i];
	mean /= mean_len;
	return mean;
}

void Pid_Control(void){  //控制姿态
	Update_attitude_Angle();  //读取欧拉角
	
//	Pitch = Mean_Filter(mean_filter_pitch, Pitch);
//	Roll = Mean_Filter(mean_filter_roll, Roll);
//	Yaw = Mean_Filter(mean_filter_yaw, Yaw);
	
	if(expect.yaw - Yaw > 280)
		expect.yaw -= 360;
	else if(expect.yaw - Yaw < -280)
		expect.yaw += 360;
	
	err_pre.pitch = err.pitch;
	err_pre.roll = err.roll;
	err_pre.yaw = err.yaw;
	
	err.pitch = expect.pitch - Pitch;
	err.roll = expect.roll - Roll;
	err.yaw = expect.yaw - Yaw;
		
	//更新变速积分系数
//	vs.pitch = Update_Vs(err.pitch);
//	vs.roll = Update_Vs(err.roll);
//	vs.yaw = Update_Vs(err.yaw);
	
	//误差累计
//	err_sum.pitch += vs.pitch * err.pitch;
//	err_sum.roll += vs.roll * err.roll;
//	err_sum.yaw += vs.yaw * err.yaw;

	
	uk.pitch = kp * err.pitch + kd * (err.pitch - err_pre.pitch);
	uk.roll = kp * err.roll + kd * (err.roll - err_pre.roll);
	uk.yaw = kp * err.yaw + kd * (err.yaw - err_pre.yaw);
	
	offset_f = -uk.pitch - uk.yaw;
	offset_b = uk.pitch - uk.yaw;
	offset_l = -uk.roll + uk.yaw;
	offset_r = uk.roll + uk.yaw;	
	
	//对offset限幅
//	offset_f = Limit_Offset(offset_f);
//	offset_b = Limit_Offset(offset_b);
//	offset_l = Limit_Offset(offset_l);
//	offset_r = Limit_Offset(offset_r);
}

short Median(float arr[]){  //对数组排序并输出中位数
	short i, j, tmp;
	for(i = 0;i < median_len - 1; i++)  //冒泡排序
		for(j = 0; j < median_len - 1 - i; j++)
			if(arr[j] > arr[j + 1]){
				tmp = arr[j];
				arr[j] = arr[j + 1];
				arr[j + 1] = tmp;
			}
	return arr[median_len / 2];
}

void Expect_Update(void){
	Remote_Resolve();  //更新遥控器信号
	median_filter[median_i] += remote_turn / 100;
	median_i += 1;
	if(median_i >= median_len){
		expect.yaw = Median(median_filter);  //中值滤波
		median_i = 0;
	}
	expect.pitch = init.pitch + 10.0 * remote_fb / 500;
	expect.roll = init.roll + 10.0 * remote_lr / 500;
}

short Limit_Amplitude(short duty_cycle){  //对占空比限幅
	if(duty_cycle > 1000)
		duty_cycle = 1000;
	else if(duty_cycle < 500)
		duty_cycle = 500;
	return duty_cycle;
}

void Motor_F(short duty_cycle){
	duty_cycle = Limit_Amplitude(duty_cycle);
	TIM_SetCompare1(TIM2,(int)(10000 - 10*duty_cycle));
}

void Motor_B(short duty_cycle){
	duty_cycle = Limit_Amplitude(duty_cycle);
	TIM_SetCompare2(TIM2,(int)(10000 - 10*duty_cycle));
}

void Motor_L(short duty_cycle){
	duty_cycle = Limit_Amplitude(duty_cycle);
	TIM_SetCompare3(TIM2,(int)(10000 - 10*duty_cycle));
}

void Motor_R(short duty_cycle){
	duty_cycle = Limit_Amplitude(duty_cycle);
	TIM_SetCompare4(TIM2,(int)(10000 - 10*duty_cycle));
}

void Move(void){  //运动控制
	Expect_Update();  //更新期望姿态
	Pid_Control();  //更新陀螺仪信息并PID控制平衡
	
	if(remote_mode[1] == '2'){  //紧急停止（右手短按），且重新初始化
		remote_base = 500;
		offset_f = 0;
		offset_b = 0;
		offset_l = 0;
		offset_r = 0;
		
	}
	
	//结束前一定记得把这四个函数给改回去啊！！！
	Motor_F(remote_base + offset_f);
	Motor_B(remote_base + offset_b);
	Motor_L(remote_base + offset_l);
	Motor_R(remote_base + offset_r);

}
