#ifndef __CUSTOMDEF_H
#define __CUSTOMDEF_H

#include "stm32f10x.h"
#include <stdio.h>

#define A     GPIOA
#define B     GPIOB
#define C     GPIOC
#define Pin0  GPIO_Pin_0
#define Pin1  GPIO_Pin_1
#define Pin2  GPIO_Pin_2
#define Pin3  GPIO_Pin_3
#define Pin4  GPIO_Pin_4
#define Pin5  GPIO_Pin_5
#define Pin6  GPIO_Pin_6
#define Pin7  GPIO_Pin_7
#define Pin8  GPIO_Pin_8
#define Pin9  GPIO_Pin_9
#define Pin10 GPIO_Pin_10
#define Pin11 GPIO_Pin_11
#define Pin12 GPIO_Pin_12
#define Pin13 GPIO_Pin_13
#define Pin14 GPIO_Pin_14
#define Pin15 GPIO_Pin_15

int fputc(int ch, FILE* f);

void setGPIO_Structure(
    GPIO_InitTypeDef* Initstructure, // GPIO初始化结构体
    GPIO_TypeDef* GPIOx,             // GPIO端口
    GPIOMode_TypeDef GPIOMode,       // GPIO模式
    u16 GPIOPin,                     // GPIO引脚
    GPIOSpeed_TypeDef GPIOSpeed      // GPIO速度
);

void setEXTI_Structure(
    EXTI_InitTypeDef* Initstructure,  // 外部中断初始化结构体
    u32 EXTI_Line,                    // 中断线
    EXTIMode_TypeDef EXTI_Mode,       // 中断模式
    EXTITrigger_TypeDef EXTI_Trigger, // 触发方式
    FunctionalState EXTI_LineCmd      // 外部中断使能
);

void setNVIC_Structure(
    NVIC_InitTypeDef* Initstructure,      // NVIC初始化结构体
    u8 NVIC_IRQChannel,                   // 中断通道
    u8 NVIC_IRQChannelPreemptionPriority, // 抢占优先级
    u8 NVIC_IRQChannelSubPriority,        // 响应优先级
    FunctionalState NVIC_IRQChannelCmd    // 中断使能
);

void setUSART_Structure(
    USART_InitTypeDef* Initstructure,   // USART初始化结构体
    USART_TypeDef* USARTx,              // USART通道
    u32 USART_BaudRate,                 // 通信波特率
    u16 USART_WordLength,               // 字长
    u16 USART_StopBits,                 // 停止位
    u16 USART_Parity,                   // 奇偶校验控制选择
    u16 USART_Mode,                     // 模式
    u16 USART_HardwareFlowControl       // 硬件流控制
);

void setTIM_Structure(
    TIM_TimeBaseInitTypeDef* TIM_TimeBaseStructure, // TIM初始化结构体
    TIM_TypeDef* TIMx,                              // TIM通道
    u16 TIM_Prescaler,                              // 预分频系数
    u16 TIM_Period,                                 // 自动重装载值
    u16 TIM_CounterMode,                            // 计数器计数模式
    u16 TIM_ClockDivision,                          // 时钟分割
    u8 TIM_RepetitionCounter                        // 重复计数器
);

void setDMA_Structure(
    DMA_InitTypeDef* DMA_InitStructure, // DMA初始化结构体
    DMA_Channel_TypeDef* DMA_Channel,     // DMA通道
    u32 DMA_PeripheralBaseAddr,          // 外设基地址
    u32 DMA_MemoryBaseAddr,              // 内存基地址
    u32 DMA_DIR,                         // 数据传输方向
    u32 DMA_BufferSize,                  // 数据缓冲区大小
    u32 DMA_PeripheralInc,               // 外设地址自增
    u32 DMA_MemoryInc,                   // 内存地址自增
    u32 DMA_PeripheralDataSize,          // 外设数据宽度
    u32 DMA_MemoryDataSize,              // 内存数据宽度
    u32 DMA_Mode,                        // DMA模式
    u32 DMA_Priority,                    // DMA优先级
    u32 DMA_M2M                          // DMA通道是否为内存到内存模式
);

void setADC_Structure(
    ADC_InitTypeDef* ADC_InitStructure, // ADC初始化结构体
    ADC_TypeDef* ADCx,                   // ADC通道
    u32 ADC_Mode,                        // ADC模式
    FunctionalState ADC_ScanConvMode,                // ADC扫描模式
    FunctionalState ADC_ContinuousConvMode,          // ADC连续转换模式
    u32 ADC_ExternalTrigConv,            // ADC外部触发转换源
    u32 ADC_DataAlign,                   // ADC数据对齐方式
    u8 ADC_NbrOfChannel                  // ADC通道数
);

void CLK_init(void);

void Set(GPIO_TypeDef* GPIOx, u16 GPIO_Pin);
void Reset(GPIO_TypeDef* GPIOx, u16 GPIO_Pin);

void DMA1_CH1_Init(void);

void TIM1_Init(void);

void ADC1_Init(void);

float trans_Temperature(float voltage);

float get_adc_temperature(void);

#endif
