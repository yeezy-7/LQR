#ifndef __MOTOR_H
#define __MOTOR_H
#include <sys.h>	 
#define PWMB   TIM1->CCR4  //PA11
#define BIN2   PBout(12)
#define BIN1   PBout(13)
#define AIN2   PBout(15)
#define AIN1   PBout(14)
#define PWMA   TIM1->CCR1  //PA8

void MiniBalance_PWM_Init(u16 arr,u16 psc);
void MiniBalance_Motor_Init(void);
#endif
