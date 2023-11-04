#include "stm32f10x.h"
#include "customdef.h"

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

void Set(GPIO_TypeDef* GPIOx, u16 GPIOPin)
{
    GPIO_SetBits(GPIOx, GPIOPin);
}

void Reset(GPIO_TypeDef* GPIOx, u16 GPIOPin)
{
    GPIO_ResetBits(GPIOx, GPIOPin);
}

void INT_init(void)
{
    // 使能AFIO时钟和GPIO时钟
    RCC_APB2PeriphClockCmd(
        RCC_APB2Periph_AFIO |
        RCC_APB2Periph_GPIOB |
        RCC_APB2Periph_GPIOA,
        ENABLE
    );
    // 启用TIM2时钟
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

    GPIO_InitTypeDef GPIO_InitStructure;
    EXTI_InitTypeDef EXTI_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;

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
    TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);

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

    // 指定外部中断源为GPIOA8和GPIOB10
    // PA8
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource8);
    // PB10
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource10);

    setEXTI_Structure(
        &EXTI_InitStructure,
        EXTI_Line8 | EXTI_Line10,   // EXTI_Line0 and EXTI_Line8 and EXTI_Line10
        EXTI_Mode_Interrupt,        // Interrupt mode
        EXTI_Trigger_Falling,       // Trigger on falling edge
        ENABLE                      // Enable EXTI_Line8 and EXIT_Line10
    );

    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    // 配置NVIC为2位抢占优先级，2位响应优先级

    setNVIC_Structure(
        &NVIC_InitStructure,
        EXTI9_5_IRQn,
        1,
        1,
        ENABLE
    ); // Enable interrupt 8

    setNVIC_Structure(
        &NVIC_InitStructure,
        EXTI15_10_IRQn,
        1,
        2,
        ENABLE
    ); // Enable interrupt 10

    setNVIC_Structure(
        &NVIC_InitStructure,
        TIM2_IRQn,
        2,
        1,
        ENABLE
    ); // Enable TIM2 interrupt

}

void EXTI9_5_IRQHandler(void)
{
    if (EXTI_GetITStatus(EXTI_Line8) == SET)
    {
        Reset(GPIOB, Pin2);                 // 点亮LED2
        EXTI_ClearITPendingBit(EXTI_Line8); // Clear interrupt flag
    }
}

void EXTI15_10_IRQHandler(void)
{
    if (EXTI_GetITStatus(EXTI_Line10) == SET)
    {
        Set(GPIOB, Pin2);                    // 熄灭LED2
        EXTI_ClearITPendingBit(EXTI_Line10); // Clear interrupt flag
    }
}

void TIM2_IRQHandler(void)
{
    if (GPIO_ReadOutputDataBit(GPIOC, Pin13) == Bit_RESET)
    {
        Set(GPIOC, Pin13);
    }
    else
    {
        Reset(GPIOC, Pin13);
    }
    TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
}

void delays(int sec)
{
    for (int i = 0; i < sec; i++)
    {
        while (TIM_GetFlagStatus(TIM2, TIM_FLAG_Update) == RESET);
        TIM_ClearFlag(TIM2, TIM_FLAG_Update);
    }
}
