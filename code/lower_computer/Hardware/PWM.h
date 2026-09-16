#ifndef __PWM_H
#define __PWM_H

#include "stm32f10x.h"

void PWM_Init(void);
void PWM_SetFrequency(uint16_t arr);
void PWM_Start(void);
void PWM_Stop(void);

#endif
