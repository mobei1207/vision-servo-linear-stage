#include "Motor.h"
#include "PWM.h"
#include <stdio.h>

// ========== 全局变量定义（只在此处定义一次）==========
volatile uint8_t motor_running = 0;
volatile uint8_t motor_dir = 1;      // 1:正转, 0:反转
volatile int32_t motor_position = 0;
volatile uint8_t speed_level = 5;

// 速度档位对应的ARR值（细分3200步/圈，TIM1频率1MHz）
// 转速范围：15~200 rpm
const uint16_t speed_arr[11] = {
    0,      // 索引0不用
    1250,   // 1档: 15 rpm
    750,    // 2档: 25 rpm
    536,    // 3档: 35 rpm
    375,    // 4档: 50 rpm
    268,    // 5档: 70 rpm
    208,    // 6档: 90 rpm
    171,    // 7档: 110 rpm
    134,    // 8档: 140 rpm
    110,    // 9档: 170 rpm
    94      // 10档: 200 rpm
};

// ========== 函数实现 ==========
void Motor_Init(void)
{
    // 开启GPIO和TIM1时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_TIM1, ENABLE);
    
    GPIO_InitTypeDef GPIO_InitStruct;
    
    // PA8 - TIM1_CH1 复用推挽
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_8;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStruct);
    
    // PB5 - 方向控制
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_5;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStruct);
    GPIO_ResetBits(GPIOB, GPIO_Pin_5);   // 默认正转（假设低电平正转）
    
    // PB10, PB11 - 光电门输入（上拉）
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(GPIOB, &GPIO_InitStruct);
    
    // 初始化PWM
    PWM_Init();
    
    // 设置默认速度
    Motor_SetSpeed(speed_level);
}

void Motor_SetSpeed(uint8_t level)
{
    if (level < 1) level = 1;
    if (level > 10) level = 10;
    speed_level = level;
    uint16_t arr = speed_arr[level];
    PWM_SetFrequency(arr);
}

void Motor_Start(void)
{
    // 设置方向
    if (motor_dir == 1) {
        GPIO_ResetBits(GPIOB, GPIO_Pin_5);   // 低电平正转（根据实际调整）
    } else {
        GPIO_SetBits(GPIOB, GPIO_Pin_5);
    }
		
    PWM_Start();
    motor_running = 1;
}

void Motor_Stop(void)
{
    PWM_Stop();
    motor_running = 0;
    // 不操作EN，DM542默认使能，保持锁死（刹车）
}

void Motor_EmergencyStop(void)
{
    PWM_Stop();
    motor_running = 0;
	printf("Motor_Stop called, PWM should be stopped\r\n");
}
