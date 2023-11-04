#ifndef __CUSTOMDEF_H
#define __CUSTOMDEF_H

#include "stm32f10x.h"
#include "stm32f10x_spi.h"

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

// void setSPI_InitTypeDef(
//     SPI_InitTypeDef *Initstructure,
//     SPI_TypeDef *SPIx,
//     u16 SPI_Mode,
//     u16 SPI_Direction,
//     u16 SPI_DataSize,
//     u16 SPI_FirstBit,
//     u16 SPI_BaudRatePrescaler,
//     u16 SPI_CPOL,
//     u16 SPI_CPHA,
//     u16 SPI_NSS,
//     u16 SPI_CRCPolynomial
// );

void Set(GPIO_TypeDef* GPIOx, u16 GPIO_Pin);
void Reset(GPIO_TypeDef* GPIOx, u16 GPIO_Pin);

#endif
