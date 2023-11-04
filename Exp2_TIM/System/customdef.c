#include "stm32f10x.h"
#include "customdef.h"
// #include "spi.h"

/**
 * @brief  设置GPIO初始化结构体
 * @param  Initstructure GPIO初始化结构体
 * @param  GPIOx GPIO端口
 * @param  GPIOMode GPIO模式
 * @param  GPIOPin GPIO引脚
 * @param  GPIOSpeed GPIO速度
 * @retval None
 */
void setGPIO_Structure(GPIO_InitTypeDef* Initstructure, GPIO_TypeDef* GPIOx, GPIOMode_TypeDef GPIOMode, u16 GPIOPin, GPIOSpeed_TypeDef GPIOSpeed)
{
    Initstructure->GPIO_Mode = GPIOMode;
    Initstructure->GPIO_Pin = GPIOPin;
    Initstructure->GPIO_Speed = GPIOSpeed;
    GPIO_Init(GPIOx, Initstructure);
}

/**
 * @brief  设置外部中断初始化结构体
 * @param  Initstructure EXTI初始化结构体
 * @param  EXTI_Line 外部中断线
 * @param  EXTI_Mode 外部中断模式
 * @param  EXTI_Trigger 触发方式
 * @param  EXTI_LineCmd 外部中断使能
 * @retval None
 */
void setEXTI_Structure(EXTI_InitTypeDef* Initstructure, u32 EXTI_Line, EXTIMode_TypeDef EXTI_Mode, EXTITrigger_TypeDef EXTI_Trigger, FunctionalState EXTI_LineCmd)
{
    Initstructure->EXTI_Line = EXTI_Line;
    Initstructure->EXTI_Mode = EXTI_Mode;
    Initstructure->EXTI_Trigger = EXTI_Trigger;
    Initstructure->EXTI_LineCmd = EXTI_LineCmd;
    EXTI_Init(Initstructure);
}

/**
 * @brief 设置NVIC初始化结构体
 * @param Initstructure NVIC初始化结构体
 * @param NVIC_IRQChannel 中断通道
 * @param NVIC_IRQChannelPreemptionPriority 抢占优先级
 * @param NVIC_IRQChannelSubPriority 响应优先级
 * @param NVIC_IRQChannelCmd 使能状态
 * @retval None
 */
void setNVIC_Structure(NVIC_InitTypeDef* Initstructure, u8 NVIC_IRQChannel, u8 NVIC_IRQChannelPreemptionPriority, u8 NVIC_IRQChannelSubPriority, FunctionalState NVIC_IRQChannelCmd)
{
    Initstructure->NVIC_IRQChannel = NVIC_IRQChannel;
    Initstructure->NVIC_IRQChannelPreemptionPriority = NVIC_IRQChannelPreemptionPriority;
    Initstructure->NVIC_IRQChannelSubPriority = NVIC_IRQChannelSubPriority;
    Initstructure->NVIC_IRQChannelCmd = NVIC_IRQChannelCmd;
    NVIC_Init(Initstructure);
}

/**
 * @brief 设置USART初始化结构体
 * @param Initstructure USART初始化结构体
 * @param USARTx USART通道
 * @param USART_BaudRate USART波特率
 * @param USART_WordLength USART字长
 * @param USART_StopBits USART停止位
 * @param USART_Parity USART校验位
 * @param USART_Mode USART模式
 * @param USART_HardwareFlowControl USART硬件流控制
 */
void setUSART_Structure(USART_InitTypeDef* Initstructure, USART_TypeDef* USARTx, u32 USART_BaudRate, u16 USART_WordLength, u16 USART_StopBits, u16 USART_Parity, u16 USART_Mode, u16 USART_HardwareFlowControl)
{
    Initstructure->USART_BaudRate = USART_BaudRate;
    Initstructure->USART_WordLength = USART_WordLength;
    Initstructure->USART_StopBits = USART_StopBits;
    Initstructure->USART_Parity = USART_Parity;
    Initstructure->USART_Mode = USART_Mode;
    Initstructure->USART_HardwareFlowControl = USART_HardwareFlowControl;
    USART_Init(USARTx, Initstructure);
    USART_Cmd(USARTx, ENABLE);
}

/**
 * @brief 设置TIM初始化结构体
 * @param TIM_TimeBaseStructure TIM初始化结构体
 * @param TIMx TIM通道
 * @param TIM_Prescaler 预分频值
 * @param TIM_Period 自动重装载值
 * @param TIM_CounterMode 计数模式
 * @param TIM_ClockDivision 时钟分频
 * @param TIM_RepetitionCounter 重复计数器
 * @retval None
 */
void setTIM_Structure(TIM_TimeBaseInitTypeDef* TIM_TimeBaseStructure, TIM_TypeDef* TIMx, u16 TIM_Prescaler, u16 TIM_Period, u16 TIM_CounterMode, u16 TIM_ClockDivision, u8 TIM_RepetitionCounter)
{
    TIM_TimeBaseStructure->TIM_Prescaler = TIM_Prescaler - 1;
    TIM_TimeBaseStructure->TIM_Period = TIM_Period - 1;
    TIM_TimeBaseStructure->TIM_CounterMode = TIM_CounterMode;
    TIM_TimeBaseStructure->TIM_ClockDivision = TIM_ClockDivision;
    TIM_TimeBaseStructure->TIM_RepetitionCounter = TIM_RepetitionCounter;
    TIM_TimeBaseInit(TIMx, TIM_TimeBaseStructure);

    TIM_Cmd(TIMx, ENABLE);
}
// void setSPI_InitTypeDef(SPI_InitTypeDef *Initstructure, SPI_TypeDef *SPIx, u16 SPI_Mode, u16 SPI_Direction, u16 SPI_DataSize, u16 SPI_FirstBit, u16 SPI_BaudRatePrescaler, u16 SPI_CPOL, u16 SPI_CPHA, u16 SPI_NSS, u16 SPI_CRCPolynomial)
// {
//     Initstructure->SPI_Mode              = SPI_Mode;              // SPI模式:SPI_Mode_Master/SPI_Mode_Slave
//     Initstructure->SPI_Direction         = SPI_Direction;         // 数据传输方向:SPI_Direction_2Lines_FullDuplex/SPI_Direction_2Lines_RxOnly/SPI_Direction_1Line_Rx/SPI_Direction_1Line_Tx
//     Initstructure->SPI_DataSize          = SPI_DataSize;          // 数据位长度:SPI_DataSize_8b/SPI_DataSize_16b
//     Initstructure->SPI_FirstBit          = SPI_FirstBit;          // 数据传输从MSB位还是LSB位开始:SPI_FirstBit_MSB/SPI_FirstBit_LSB
//     Initstructure->SPI_BaudRatePrescaler = SPI_BaudRatePrescaler; // 波特率预分频值
//     Initstructure->SPI_CPOL              = SPI_CPOL;              // 时钟极性
//     Initstructure->SPI_CPHA              = SPI_CPHA;              // 时钟极性和相位
//     Initstructure->SPI_NSS               = SPI_NSS;               // 选择软件控制的NSS信号
//     Initstructure->SPI_CRCPolynomial     = SPI_CRCPolynomial;     // CRC多项式寄存器
//     SPI_Init(SPIx, Initstructure);                               // 初始化SPI外设
//     SPI_Cmd(SPIx, ENABLE);                                        // 使能SPI外设
//     SPI_CS_W(1);                                                  // SPI片选置高
// }

void Set(GPIO_TypeDef* GPIOx, u16 GPIOPin)
{
    GPIO_SetBits(GPIOx, GPIOPin);
}

void Reset(GPIO_TypeDef* GPIOx, u16 GPIOPin)
{
    GPIO_ResetBits(GPIOx, GPIOPin);
}
