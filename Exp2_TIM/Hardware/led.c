#include "stm32f10x.h"
#include "customdef.h"
#include "delay.h"
#include "led.h"

void LEDA_Init(void)
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    GPIO_InitTypeDef GPIO_InitStructure;
    setGPIO_Structure(
                        &GPIO_InitStructure,
                        A,
                        GPIO_Mode_Out_PP,
                        Pin1|Pin2,
                        GPIO_Speed_2MHz
                                        );
}

void HolyShit()
{
    TestPre();
    GPIO_InitTypeDef GPIO_InitStructure;
    setGPIO_Structure(&GPIO_InitStructure, A, GPIO_Mode_Out_PP, GPIO_Pin_All, GPIO_Speed_2MHz);
    setGPIO_Structure(&GPIO_InitStructure, B, GPIO_Mode_Out_PP, GPIO_Pin_All, GPIO_Speed_2MHz);
    setGPIO_Structure(&GPIO_InitStructure, C, GPIO_Mode_Out_PP, GPIO_Pin_All, GPIO_Speed_2MHz);

    while (1)
    {
        DSideLight();
    }
}

void TestPre()
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);
    // 这里使用AFIO是为了使能B3、B4、A13、A14、A15
    GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);
    // 同上
}

void streamLight(GPIO_TypeDef* GPIOx, u16 GPIOPin, int time)
{
    GPIO_SetBits(GPIOx, GPIOPin);
    Delay_ms(time);
    GPIO_ResetBits(GPIOx, GPIOPin);
}


void streamLightOneTest()
{
    streamLight(GPIOC, Pin13, 50);
    streamLight(GPIOC, Pin15, 50);
    streamLight(GPIOA, Pin1, 50);
    streamLight(GPIOA, Pin3, 50);
    streamLight(GPIOA, Pin5, 50);
    streamLight(GPIOA, Pin7, 50);
    streamLight(GPIOB, Pin1, 50);
    streamLight(GPIOB, Pin11, 50);

    streamLight(GPIOB, Pin12, 50);
    streamLight(GPIOB, Pin14, 50);
    streamLight(GPIOA, Pin8, 50);
    streamLight(GPIOA, Pin10, 50);
    streamLight(GPIOA, Pin12, 50);
    streamLight(GPIOB, Pin3, 50);
    streamLight(GPIOB, Pin5, 50);
    streamLight(GPIOB, Pin7, 50);
    streamLight(GPIOB, Pin9, 50);
}

void SetGSide()
{
    Set(B, Pin12);
    Set(B, Pin14);
    Set(A, Pin8);
    Set(A, Pin10);
    Set(A, Pin12);
    Set(B, Pin3);
    Set(B, Pin5);
    Set(B, Pin7);
    Set(B, Pin9);
}

void ResetGSide()
{
    Reset(B, Pin12);
    Reset(B, Pin14);
    Reset(A, Pin8);
    Reset(A, Pin10);
    Reset(A, Pin12);
    Reset(B, Pin3);
    Reset(B, Pin5);
    Reset(B, Pin7);
    Reset(B, Pin9);
}

void SetBSide()
{
    Set(C, Pin13);
    Set(C, Pin15);
    Set(A, Pin1);
    Set(A, Pin3);
    Set(A, Pin5);
    Set(A, Pin7);
    Set(B, Pin1);
    Set(B, Pin11);
}

void ResetBSide()
{
    Reset(C, Pin13);
    Reset(C, Pin15);
    Reset(A, Pin1);
    Reset(A, Pin3);
    Reset(A, Pin5);
    Reset(A, Pin7);
    Reset(B, Pin1);
    Reset(B, Pin11);
}

void DSideLight()
{
    ResetGSide();ResetBSide();

    ResetBSide();Delay_ms(300);

    SetBSide();Delay_ms(300);

    ResetBSide();Delay_ms(300);

    SetBSide();Delay_ms(300);

    ResetBSide();Delay_ms(300);

    ResetGSide();Delay_ms(300);

    SetGSide();Delay_ms(300);

    ResetGSide();Delay_ms(300);

    SetGSide();Delay_ms(300);

    ResetGSide();Delay_ms(300);
}

void Turn_LED(GPIO_TypeDef *GPIOx, u16 GPIO_Pin)
{
    if(GPIO_ReadOutputDataBit(GPIOx, GPIO_Pin) == 0)
    {
        Set(GPIOx, GPIO_Pin);
    }
    else
    {
        Reset(GPIOx, GPIO_Pin);
    }
}
