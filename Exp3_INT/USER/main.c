#include "stm32f10x.h"
#include <stdio.h>
#include "customdef.h"
#include "key.h"

/*
    实验三
    内容：
        根据实验原理图配置对应的GPIO口和定时器，实现下述功能:
            1.配置LED1、LED2和串口相关的IO功能；
            2.配置TIM2，根据系统72MHz时钟，配置定时周期为1秒，向上计数模式；
            3.配置TIM2的中断，在中断响应函数中翻转PC13的输出状态，实现LED1的闪烁；
            4.配置PA8、PB10为上拉输入，使能AFIO；
            5.配置EXTI9_5、EXTI15_10的中断优先级；
            6.配置PA8、PB10的中断模式为下降沿中断；
            7.在PA8的中断响应函数中点亮LED2。
            8.在PB10的中断响应函数中熄灭LED2。
*/

int fputc(int ch, FILE* f)
{
    while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);
    USART_SendData(USART1, (uint8_t)ch);

    return ch;
}

int main(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;

    // 启用GPIOA、GPIOB、GPIOC时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);

    // 启用USART1时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);

    // 启用PC13、PB2
    // PC13 设置为推挽输出
    setGPIO_Structure(
        &GPIO_InitStructure,
        C,
        GPIO_Mode_Out_PP,
        Pin13,
        GPIO_Speed_2MHz
    );

    // PB2 设置为推挽输出
    setGPIO_Structure(
        &GPIO_InitStructure,
        B,
        GPIO_Mode_Out_PP,
        Pin2,
        GPIO_Speed_2MHz
    );

    // 启用PA9、PA10
    setGPIO_Structure(
        &GPIO_InitStructure,
        A,
        GPIO_Mode_AF_PP,
        Pin9,
        GPIO_Speed_10MHz
    );

    setUSART_Structure(
        &USART_InitStructure,
        USART1,
        9600,
        USART_WordLength_8b,
        USART_StopBits_1,
        USART_Parity_No,
        USART_Mode_Tx,
        USART_HardwareFlowControl_None
    );
    printf("USART1 initialized.\n");

    // 初始化LED灯状态为灭
    // 因为使用的是上接VCC，所以低电平亮，高电平灭
    GPIO_SetBits(C, Pin13);
    GPIO_SetBits(B, Pin2);

    // 使能各个中断
    INT_init();
    printf("TIM2 Init Completely\r\n");
    printf("INT Init Completely\r\n");

    while (1)
    {

    }
}
