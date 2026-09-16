#include "stm32f10x.h"
#include "Delay.h"
#include "Motor.h"
#include "Serial.h"
#include <stdio.h>

// ========== 仅在此处定义 main.c 专用的全局变量 ==========
volatile uint8_t homing_flag = 0;   // 调零进行中标志

// 定义光电门触发电平（NPN常开：触发为低电平）
#define LIMIT_TRIGGER_LEVEL   RESET

// 函数声明（本文件内部函数）
void process_command(uint8_t cmd, uint8_t data);
void check_limits(void);
void homing_process(void);

// ========== 主函数 ==========
int main(void)
{
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);   // 设置中断优先级分组
    
    Delay_ms(10);
    Motor_Init();
    Serial_Init();
	  
	  homing_flag = 1;
    motor_dir = 0;           // 反转找零点
    Motor_SetSpeed(3);       // 低速
    Motor_Start();
    printf("Auto homing started\r\n");
	  
    printf("\r\nSTM32 Stepper Motor Controller Ready.\r\n");
    printf("Commands: 0x01=Forward, 0x02=Reverse, 0x03=Stop, 0x04=Emergency, 0x05=Homing\r\n");
    printf("Speed level default: %d\r\n", speed_level);
    
    while (1)
    {
			uint8_t pb10 = GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_10);
    uint8_t pb11 = GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_11);
    printf("PB10=%d, PB11=%d\r\n", pb10, pb11);
    Delay_ms(500);  // 每500ms打印一次
			
        // 调零状态机
        if (homing_flag) {
            homing_process();
            Delay_ms(10);
            continue;
        }
        
        // 限位保护（仅在电机运行时）
        if (motor_running) {
            check_limits();
        }
        
        Delay_ms(10);
    }
		
}

// ========== 串口命令处理（由串口中断调用）==========
void process_command(uint8_t cmd, uint8_t data)
{
    switch (cmd) {
        case 0x01:  // 正转
            if (!homing_flag) {
                if (data >= 1 && data <= 10) {
                    speed_level = data;
                    Motor_SetSpeed(speed_level);
                }
                motor_dir = 1;
                Motor_Start();
                printf("Forward, speed level: %d\r\n", speed_level);
            }
            break;
        case 0x02:  // 反转
            if (!homing_flag) {
                if (data >= 1 && data <= 10) {
                    speed_level = data;
                    Motor_SetSpeed(speed_level);
                }
                motor_dir = 0;
                Motor_Start();
                printf("Reverse, speed level: %d\r\n", speed_level);
            }
            break;
        case 0x03:  // 停止
            Motor_Stop();
            printf("Stop\r\n");
            break;
        case 0x04:  // 急停
            Motor_EmergencyStop();
            printf("Emergency Stop!\r\n");
            break;
        case 0x05:  // 调零
            if (!homing_flag) {
                homing_flag = 1;
                motor_dir = 0;           // 反转寻找零点
                Motor_SetSpeed(3);       // 使用低速（35 rpm）调零
                Motor_Start();
                printf("Homing start...\r\n");
            }
            break;
        default:
            printf("Unknown command: 0x%02X\r\n", cmd);
            break;
    }
}

// ========== 限位检测 ==========
void check_limits(void)
{
    // 正转时检测正限位（PB11）
    if (motor_dir == 1) {
        if (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_11) == LIMIT_TRIGGER_LEVEL) {
            Motor_EmergencyStop();
            printf("Positive limit triggered!\r\n");
        }
    } 
    // 反转时检测负限位/零点（PB10）
    else {
        if (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_10) == LIMIT_TRIGGER_LEVEL) {
            Motor_EmergencyStop();
            printf("Negative limit triggered!\r\n");
        }
    }
}

// ========== 调零过程 ==========
void homing_process(void)
{
    // 检测零点光电门（PB10）触发
    if (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_10) == LIMIT_TRIGGER_LEVEL) {
        Motor_Stop();
        motor_position = 0;
        homing_flag = 0;
        printf("Homing complete, position = 0\r\n");
    }
    // 若在调零过程中意外触发了正限位（安全保护）
    if (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_11) == LIMIT_TRIGGER_LEVEL) {
        Motor_EmergencyStop();
        homing_flag = 0;
        printf("Homing failed: positive limit reached\r\n");
    }
}
