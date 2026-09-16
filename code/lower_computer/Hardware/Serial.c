#include "Serial.h"
#include "Motor.h"   // 为了使用 motor_xxx 变量（但仅作为外部引用）

// 接收缓冲区（静态，仅本文件使用）
static uint8_t rx_buf[4];
static uint8_t rx_index = 0;

// 外部函数声明（定义在main.c中）
extern void process_command(uint8_t cmd, uint8_t data);

void Serial_Init(void)
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_USART1, ENABLE);
    
    GPIO_InitTypeDef GPIO_InitStruct;
    // PA9 TX
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_9;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStruct);
    // PA10 RX
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &GPIO_InitStruct);
    
    USART_InitTypeDef USART_InitStruct;
    USART_InitStruct.USART_BaudRate = 115200;
    USART_InitStruct.USART_WordLength = USART_WordLength_8b;
    USART_InitStruct.USART_StopBits = USART_StopBits_1;
    USART_InitStruct.USART_Parity = USART_Parity_No;
    USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStruct.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
    USART_Init(USART1, &USART_InitStruct);
    
    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    NVIC_EnableIRQ(USART1_IRQn);
    NVIC_SetPriority(USART1_IRQn, 0);
    
    USART_Cmd(USART1, ENABLE);
}

void Serial_SendByte(uint8_t ch)
{
    USART_SendData(USART1, ch);
    while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
}

void Serial_SendString(char* str)
{
    while (*str) {
        Serial_SendByte((uint8_t)*str++);
    }
}

// 重定向printf
int fputc(int ch, FILE *f)
{
    Serial_SendByte((uint8_t)ch);
    return ch;
}

// 串口中断服务函数
void USART1_IRQHandler(void)
{
    if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET) {
        uint8_t data = USART_ReceiveData(USART1);
        rx_buf[rx_index++] = data;
        if (rx_index == 1 && rx_buf[0] != 0xAA) {
            rx_index = 0;   // 帧头错误
        }
        if (rx_index == 4) {
            uint8_t sum = (rx_buf[0] + rx_buf[1] + rx_buf[2]) & 0xFF;
            if (sum == rx_buf[3]) {
                process_command(rx_buf[1], rx_buf[2]);
            }
            rx_index = 0;
        }
        if (rx_index >= 4) rx_index = 0;
    }
}
