#include "stm32f10x.h"
#include <stdio.h>
#include "customdef.h"
#include "key.h"
#include "delay.h"
#include "led.h"

/*
    实验一
    内容：
        根据原理图
        PA8  -> KEY0        上拉输入
        PB10 -> KEY1        上拉输入
        PC13 -> LED0
        PB2  -> LED1

        其次设置USART接口
        PA9  -> USART1_TX
        PA10 -> USART1_RX

        ①：
            按下KEY0，LED0灭，松开KEY0，LED0亮
            按下KEY1，LED1灭，松开KEY1，LED1亮
        ②：
            没有按下按键，两个LED灯都亮
            按下KEY0，两个灯同步周期性闪烁
            按下KEY1，两个灯反向周期性闪烁
*/
void Key_GetNumEXP1_1(GPIO_TypeDef *GPIOx, u16 GPIO_Pin);
void Key_GetNumEXP1_2(GPIO_TypeDef *GPIOx, u16 GPIO_Pin);

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

    // setGPIO_Structure(
    //     &GPIO_InitStructure,
    //     A,
    //     GPIO_Mode_AF_PP,
    //     Pin10,
    //     GPIO_Speed_10MHz
    // );

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

//     while (1)
//     {
//         // ①
//         // Key_GetNumPro1_1(A, Pin8);
//         // Key_GetNumPro1_1(B, Pin10);
//         // 更简单的方法
//         if (GPIO_ReadInputDataBit(A, Pin8) == 0)
//         {
//             GPIO_ResetBits(C, Pin13);
//         }
//         else
//         {
//             GPIO_SetBits(C, Pin13);
//         }

//         if (GPIO_ReadInputDataBit(B, Pin10) == 0)
//         {
//             GPIO_ResetBits(B, Pin2);
//         }
//         else
//         {
//             GPIO_SetBits(B, Pin2);
//         }
//     }

    while (1)
    {
        // // ②
				GPIO_ResetBits(C, Pin13);
				GPIO_ResetBits(B, Pin2);
        // Key_GetNumPro1_2(A, Pin8)
        // Key_GetNumPro1_2(B, Pin10)
        // 更简单的方法
        if (GPIO_ReadInputDataBit(A, Pin8) == 0)
        {
            GPIO_ResetBits(C, Pin13);
            GPIO_ResetBits(B, Pin2);
            Delay_ms(500);
            GPIO_SetBits(C, Pin13);
            GPIO_SetBits(B, Pin2);
            Delay_ms(500);
        }
        else if (GPIO_ReadInputDataBit(B, Pin10) == 0)
        {
            GPIO_SetBits(C, Pin13);
            GPIO_ResetBits(B, Pin2);
            Delay_ms(500);
            GPIO_ResetBits(C, Pin13);
            GPIO_SetBits(B, Pin2);
            Delay_ms(500);
        }
        else
        {
            GPIO_SetBits(C, Pin13);
            GPIO_SetBits(B, Pin2);
        }
    }
}

void Key_GetNumEXP1_1(GPIO_TypeDef *GPIOx, u16 GPIO_Pin)
{
    // 检查按键是否按下
	
		if(GPIOx == GPIOA && GPIO_Pin == Pin8)
		{
			if(GPIO_ReadInputDataBit(GPIOx, GPIO_Pin) == 0)
			{
					Delay_ms(20);
					// 延时20ms，防止抖动 （按下）
					while (GPIO_ReadInputDataBit(GPIOx, GPIO_Pin)==0)
					{
						GPIO_ResetBits(GPIOC, Pin13);
					};
					Delay_ms(20);
					GPIO_SetBits(GPIOC, Pin13);
					// 延时20ms，防止抖动 （松手后）
			}
		}
		if(GPIOx == GPIOB && GPIO_Pin == Pin10)
		{
			if(GPIO_ReadInputDataBit(GPIOx, GPIO_Pin) == 0)
			{
					Delay_ms(20);
					// 延时20ms，防止抖动 （按下）
					while (GPIO_ReadInputDataBit(GPIOx, GPIO_Pin)==0)
					{
						GPIO_ResetBits(GPIOB, Pin2);
					};
					Delay_ms(20);
					GPIO_SetBits(GPIOB, Pin2);
					// 延时20ms，防止抖动 （松手后）
			}
		}
}

void Key_GetNumEXP1_2(GPIO_TypeDef *GPIOx, u16 GPIO_Pin)
{
    // 检查按键是否按下
	
		if(GPIOx == GPIOA && GPIO_Pin == Pin8)
		{
			if(GPIO_ReadInputDataBit(GPIOx, GPIO_Pin) == 0)
			{
					Delay_ms(20);
					// 延时20ms，防止抖动 （按下）
					while (GPIO_ReadInputDataBit(GPIOx, GPIO_Pin)==0)
					{
            GPIO_ResetBits(C, Pin13);
            GPIO_ResetBits(B, Pin2);
            Delay_ms(500);
            GPIO_SetBits(C, Pin13);
            GPIO_SetBits(B, Pin2);
            Delay_ms(500);
					};
					Delay_ms(20);
					GPIO_ResetBits(GPIOC, Pin13);
					GPIO_ResetBits(GPIOB, Pin2);
					// 延时20ms，防止抖动 （松手后）
			}
		}
		if(GPIOx == GPIOB && GPIO_Pin == Pin10)
		{
			if(GPIO_ReadInputDataBit(GPIOx, GPIO_Pin) == 0)
			{
					Delay_ms(20);
					// 延时20ms，防止抖动 （按下）
					while (GPIO_ReadInputDataBit(GPIOx, GPIO_Pin)==0)
					{
            GPIO_SetBits(C, Pin13);
            GPIO_ResetBits(B, Pin2);
            Delay_ms(500);
            GPIO_ResetBits(C, Pin13);
            GPIO_SetBits(B, Pin2);
            Delay_ms(500);
					};
					Delay_ms(20);
					GPIO_ResetBits(GPIOC, Pin13);
					GPIO_ResetBits(GPIOB, Pin2);
					// 延时20ms，防止抖动 （松手后）
			}
		}
}
