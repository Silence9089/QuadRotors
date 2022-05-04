#include "stm32f10x.h"
#include "motion_control.h"
#include "delay.h"
#include "pwm.h"
#include "mpu9250.h"
#include "Led.h"
#include "nRF24l01.h"

//位置式pid控制的一些参数
float kp;
float ki;
float kd;

typedef struct{
	short pitch;
	short roll;
	short yaw;
}Euler;

typedef struct{
	float pitch;
	float roll;
	float yaw;
}ft_Euler;

Euler err;  //偏差信号
Euler err_sum;
Euler init;  //上电时的欧拉角（也即飞机水平时的欧拉角）
Euler expect;  //期望的欧拉角
ft_Euler vs;  //变速积分系数
Euler uk;  //控制信号
ft_Euler pre_state;

#define p_pass ((pre_state.pitch-Pitch)>=2)||((pre_state.pitch-Pitch)<=-2)
#define r_pass ((pre_state.roll-Roll)>=2)||((pre_state.roll-Roll)<=-2)
#define y_pass ((pre_state.yaw-Yaw)>=2)||((pre_state.yaw-Yaw)<=-2)

#define ex_p_pass ((expect.pitch-Pitch)>=2)||((expect.pitch-Pitch)<=-2)
#define ex_r_pass ((expect.roll-Roll)>=2)||((expect.roll-Roll)<=-2)
#define ex_y_pass ((expect.yaw-Yaw)>=2)||((expect.yaw-Yaw)<=-2)

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
u8 remote_turn_i=0;
u8 pid_control_i=0;
u16 expect_i=0;

u8 remote_mode[4];


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

void Motion_Init(void){ //先尝试仅用比例积分控制
	int i;
	float init_pitch=0;
	float init_roll=0;
	float init_yaw=0;
	kp = 2;
	ki = 0.0005;
	kd = 0;
	uk.pitch = 0;
  uk.roll = 0;
	uk.yaw = 0;
	err_sum.pitch = 0;  
	err_sum.roll = 0;
  err_sum.yaw = 0;
	vs.pitch = 1;
	vs.roll = 1;
	vs.yaw = 1;
	offset_f = 0;  //偏移
  offset_b = 0;
  offset_l = 0; 
  offset_r = 0; 
	
//	NRF24L01_RX_Mode();
	
	for(i = 0; i < 50; i++){  
		Update_attitude_Angle();
	}
	for(i = 0; i < 50; i++){  //取50次数据的平均作为飞机当前的姿态数据
		Update_attitude_Angle();
		init_pitch += Pitch;
		init_roll += Roll;
		init_yaw += Yaw;
	}
			
	init.pitch = init_pitch / 50;
	init.roll = init_roll / 50;
	init.yaw = init_yaw / 50;
	
	pre_state.pitch = init.pitch;
  pre_state.roll = init.roll;
	pre_state.yaw = init.yaw;
	
	expect.pitch = init.pitch;
	expect.roll = init.roll;
	expect.yaw = init.yaw;
	
}

void Pid_Control(void){  //控制姿态
	Update_attitude_Angle();  //读取欧拉角
	if(ex_p_pass||ex_r_pass||ex_y_pass){
	  expect_i++;
	}
	if(p_pass||r_pass||y_pass||(expect_i>=500)){
		expect_i=0;
		
		pre_state.pitch = Pitch;
    pre_state.roll = Roll;
	  pre_state.yaw = Yaw;
		
		if(expect.yaw - Yaw > 280)
			expect.yaw -= 360;
		else if(expect.yaw - Yaw < -280)
			expect.yaw += 300;
		
		err.pitch = expect.pitch - Pitch;
		err.roll = expect.roll - Roll;
		err.yaw = expect.yaw - Yaw;
		
		err_sum.pitch += err.pitch;
		err_sum.roll += err.roll;
		err_sum.yaw += err.yaw;
		
		//变速积分上限为20度
		vs.pitch = 1.0 - 0.05 * err.pitch;
		vs.roll = 1.0 - 0.05 * err.roll;
		vs.yaw = 1.0 - 0.05 * err.yaw;
		
		//这里可以考虑加一个消除积分不灵敏区
		uk.pitch = kp * err.pitch + vs.pitch * ki * err_sum.pitch;
		uk.roll = kp * err.roll + vs.roll * ki * err_sum.roll;
		uk.yaw = kp * err.yaw + vs.yaw * ki * err_sum.yaw;
		
		offset_f = uk.pitch + uk.yaw;
		offset_b = -uk.pitch + uk.yaw;
		offset_l = uk.roll - uk.yaw;
		offset_r = -uk.roll - uk.yaw;
		
		//对offset限幅
		if(offset_f > 150)
			offset_f = 150;
		else if(offset_f < -150)
			offset_f = -150;
		
		if(offset_b > 150)
			offset_b = 150;
		else if(offset_b < -150)
			offset_b = -150;
		
		if(offset_l > 150)
			offset_l = 150;
		else if(offset_l < -150)
			offset_l = -150;
		
		if(offset_r > 150)
			offset_r = 150;
		else if(offset_r < -150)
			offset_r = -150;
		
	}		
}

void Expect_Update(void){
	Remote_Resolve();  //更新遥控器信号
	
	expect.pitch = init.pitch + 20.0 * remote_fb / 500;
	expect.roll = init.roll + 20.0 * remote_lr / 500;
	remote_turn_i++;
	if(remote_turn_i==100){
		expect.yaw += 1.0 * remote_turn / 500;
		remote_turn_i=0;
	}
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
	TIM_SetCompare1(TIM2,(int)(10000 - duty_cycle));
}

void Motor_B(short duty_cycle){
	duty_cycle = Limit_Amplitude(duty_cycle);
	TIM_SetCompare2(TIM2,(int)(10000 - duty_cycle));
}

void Motor_L(short duty_cycle){
	duty_cycle = Limit_Amplitude(duty_cycle);
	TIM_SetCompare3(TIM2,(int)(10000 - duty_cycle));
}

void Motor_R(short duty_cycle){
	duty_cycle = Limit_Amplitude(duty_cycle);
	TIM_SetCompare4(TIM2,(int)(10000 - duty_cycle));
}

void Move(void){  //运动控制
	Expect_Update();  //更新期望姿态
	Pid_Control();  //更新陀螺仪信息并PID控制平衡
	if(remote_mode[1] == '2'){
		remote_base = 500;
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
