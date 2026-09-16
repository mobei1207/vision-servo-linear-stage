#include "PWM.h"

void PWM_Init(void)
{
    // 时钟已在Motor_Init中开启，但此处再确保
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
    
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStruct;
    TIM_TimeBaseStruct.TIM_Period = 1250 - 1;   // 初始ARR
    TIM_TimeBaseStruct.TIM_Prescaler = 72 - 1;  // 72分频 -> 1MHz计数
    TIM_TimeBaseStruct.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStruct.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseStruct.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStruct);
    
    TIM_OCInitTypeDef TIM_OCStruct;
    TIM_OCStruct.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCStruct.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCStruct.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OCStruct.TIM_Pulse = (1250/2) - 1;   // 50%占空比
    TIM_OC1Init(TIM1, &TIM_OCStruct);
    
    TIM_Cmd(TIM1, ENABLE);
    TIM_CtrlPWMOutputs(TIM1, ENABLE);
    
    // 使能更新中断用于脉冲计数
    TIM_ITConfig(TIM1, TIM_IT_Update, ENABLE);
    NVIC_EnableIRQ(TIM1_UP_IRQn);
    NVIC_SetPriority(TIM1_UP_IRQn, 1);
}

void PWM_SetFrequency(uint16_t arr)
{
    if (arr < 10) arr = 10;   // 保护
    TIM1->ARR = arr - 1;
    TIM1->CCR1 = (arr / 2) - 1;
}

void PWM_Start(void)
{
    TIM_Cmd(TIM1, ENABLE);
    TIM_CtrlPWMOutputs(TIM1, ENABLE);
}

void PWM_Stop(void)
{
    TIM_Cmd(TIM1, DISABLE);
    TIM_CtrlPWMOutputs(TIM1, DISABLE);
}
