#ifndef MPU9250_H
#define MPU9250_H

void RCC_Configuration(void);
void NVIC_Configuration(void);
void WWDG_Configuration(void);
void Delay(u32 nTime);
void Delayms(vu32 m);
void Mpu9250_Init(void);
short* Accel(void);
short* Gyro(void);
short* Mag(void);


	

#endif
