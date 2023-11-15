#include "stm32f10x.h"
#include "customdef.h"
#include "Delay.h"
#include "key.h"
#include "led.h"

void Key_Init(void)
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

    GPIO_InitTypeDef GPIO_InitSturcture;
    setGPIO_Structure(&GPIO_InitSturcture, B, GPIO_Mode_IPU, Pin1|Pin11, GPIO_Speed_2MHz);
}

u8 Key_GetNum()
{
    u8 KeyNum = 0;
    // 检查按键是否按下
    if(GPIO_ReadInputDataBit(B, Pin1) == 0)
    {
        Delay_ms(20);
        // 延时20ms，防止抖动 （按下）
        while (GPIO_ReadInputDataBit(B, Pin1)==0);
        Delay_ms(20);
        // 延时20ms，防止抖动 （松手后）
        KeyNum = 1;
    }
    if(GPIO_ReadInputDataBit(B, Pin11) == 0)
    {
        Delay_ms(20);
        // 延时20ms，防止抖动 （按下）
        while (GPIO_ReadInputDataBit(B, Pin11)==0);
        Delay_ms(20);
        // 延时20ms，防止抖动 （松手后）
        KeyNum = 2;
    }
    return KeyNum;
}

u8 Key_GetNumPro(GPIO_TypeDef *GPIOx, u16 GPIO_Pin)
{
    u8 KeyNum = 0;
    // 检查按键是否按下
    if(GPIO_ReadInputDataBit(GPIOx, GPIO_Pin) == 0)
    {
        Delay_ms(20);
        // 延时20ms，防止抖动 （按下）
        while (GPIO_ReadInputDataBit(GPIOx, GPIO_Pin)==0);
        Delay_ms(20);
        // 延时20ms，防止抖动 （松手后）
        KeyNum = 1;
    }
    return KeyNum;
}
