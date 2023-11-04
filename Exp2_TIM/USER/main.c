#include "stm32f10x.h"
#include <stdio.h>
#include "customdef.h"

/*
    实验二
    内容：
        根据硬件原理图配置对应的GPIO口和定时器，
        实现下述功能：
            配置两个LED和串口相关的IO功能；         √
            配置TIM2，根据系统72MHz时钟，           √
            配置定时周期为1秒，向上计数模式；
            然后在while循环中查询等待定时器时间到，
            点亮两个LED灯，再次等待定时时间到，
            熄灭两个LED灯，循环往复。

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
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;

    // 启用GPIOA、GPIOB、GPIOC时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);

    // 启用USART1时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);

    // 启用TIM2时钟
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

    // 启用PA8、PB10、PC13、PB2

    // PA8 设置为上拉输入
    setGPIO_Structure(
        &GPIO_InitStructure,
        A,
        GPIO_Mode_IPU,
        Pin8,
        GPIO_Speed_2MHz
    );

    // PB10 设置为上拉输入
    setGPIO_Structure(
        &GPIO_InitStructure,
        B,
        GPIO_Mode_IPU,
        Pin10,
        GPIO_Speed_2MHz
    );

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

    // 使能 TIM2
    // APB1时钟最大36MHz，所以TIM2最大36MHz
    setTIM_Structure(
        &TIM_TimeBaseStructure,
        TIM2,
        36000,
        2000,
        TIM_CounterMode_Up,
        TIM_CKD_DIV1,
        0
    );

    TIM_Cmd(TIM2, ENABLE);
    printf("TIM2 Init Completely\r\n");

    /* Infinite loop */
    while (1)
    {
        while (TIM_GetFlagStatus(TIM2, TIM_FLAG_Update) == RESET);

        TIM_ClearFlag(TIM2, TIM_FLAG_Update);
        GPIO_ResetBits(GPIOB, GPIO_Pin_2);
        GPIO_ResetBits(GPIOC, GPIO_Pin_13);

        while (TIM_GetFlagStatus(TIM2, TIM_FLAG_Update) == RESET);

        TIM_ClearFlag(TIM2, TIM_FLAG_Update);
        GPIO_SetBits(GPIOB, GPIO_Pin_2);
        GPIO_SetBits(GPIOC, GPIO_Pin_13);
    }
}
