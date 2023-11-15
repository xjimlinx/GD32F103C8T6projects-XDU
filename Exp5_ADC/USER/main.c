#include "stm32f10x.h"
#include "customdef.h"

/*
    实验五
    内容：
        根据实验原理图配置对应的GPIO口和定时器，实现下述功能:
            1、初始化定时器TIM1，定时周期为500ms，
                配置CH1的PWM输出，占空比为50%；
            2、配置DMA1的通道1，实现从ADC读取20次数据到内存中，
                按半字读取；
            3、配置ADC1为独立工作模式、非扫描模式、非连续转换模式、
                外部触发方式为TIM1的捕获/比较1即ADC_ExternalTrigConv_T1_CC1，
                通道数为1；
            4、配置ADC1的Channel16为规则通道，
                采样时间为ADC_SampleTime_71Cycles5，读取温度传感器的值；
            5、启动定时器TIM1及PWM输出，监听DMA的状态，
                DMA工作完成后停止定时器，打印输出DMA搬运的ADC值；
            6、根据温度计算公式计算各个值对应的实际温度值。

*/

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

    // 启用PA9、PA10
    setGPIO_Structure(
        &GPIO_InitStructure,
        A,
        GPIO_Mode_AF_PP,
        Pin9,
        GPIO_Speed_10MHz
    );

    // 设置USART1
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
    printf("USART1 initialized.\r\n");

    // 使能各个中断
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
    ADC1_Init();

    while(1);
}
