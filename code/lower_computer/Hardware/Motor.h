#ifndef __MOTOR_H
#define __MOTOR_H

#include "stm32f10x.h"
#include <stdint.h>

// ????
void Motor_Init(void);
void Motor_SetSpeed(uint8_t level);
void Motor_Start(void);
void Motor_Stop(void);
void Motor_EmergencyStop(void);

// ??????(???Motor.c?)
extern volatile uint8_t motor_running;
extern volatile uint8_t motor_dir;
extern volatile int32_t motor_position;
extern volatile uint8_t speed_level;

// ????(????)
extern const uint16_t speed_arr[11];

#endif
