#include "stm32f10x.h"
#include "customdef.h"

DMA_InitTypeDef DMA_InitStructure;

#define ADC1_DR_Address ((u32)0x4001244C)
#define ADC1_CONV_SIZE 20

__IO u16 ADC_RegularConvertedValueTab[ADC1_CONV_SIZE];

u16 adc_count = 0;
u16 tim_count = 0;
u16 pwm_count = 0;

/**
 * @brief  重定向c库函数printf到USART1
 * @param ch
 * @param f
 * @return int
 */
int fputc(int ch, FILE* f)
{
    while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);
    USART_SendData(USART1, (uint8_t)ch);

    return ch;
}

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

}

/**
 * @brief 设置DMA初始化结构体
 * @param DMA_InitStructure DMA初始化结构体
 * @param DMA_Channel DMA通道
 * @param DMA_PeripheralBaseAddr 外设基地址
 * @param DMA_MemoryBaseAddr 内存基地址
 * @param DMA_DIR 数据传输方向
 * @param DMA_BufferSize 数据缓冲区大小
 * @param DMA_PeripheralInc 外设地址自增
 * @param DMA_MemoryInc 内存地址自增
 * @param DMA_PeripheralDataSize 外设数据宽度
 * @param DMA_MemoryDataSize 内存数据宽度
 * @param DMA_Mode DMA模式
 * @param DMA_Priority DMA优先级
 * @param DMA_M2M DMA通道是否为内存到内存模式
 */
void setDMA_Structure(DMA_InitTypeDef* DMA_InitStructure, DMA_Channel_TypeDef* DMA_Channel, u32 DMA_PeripheralBaseAddr, u32 DMA_MemoryBaseAddr, u32 DMA_DIR, u32 DMA_BufferSize, u32 DMA_PeripheralInc, u32 DMA_MemoryInc, u32 DMA_PeripheralDataSize, u32 DMA_MemoryDataSize, u32 DMA_Mode, u32 DMA_Priority, u32 DMA_M2M)
{
    DMA_InitStructure->DMA_PeripheralBaseAddr = DMA_PeripheralBaseAddr;
    DMA_InitStructure->DMA_MemoryBaseAddr = DMA_MemoryBaseAddr;
    DMA_InitStructure->DMA_DIR = DMA_DIR;
    DMA_InitStructure->DMA_BufferSize = DMA_BufferSize;
    DMA_InitStructure->DMA_PeripheralInc = DMA_PeripheralInc;
    DMA_InitStructure->DMA_MemoryInc = DMA_MemoryInc;
    DMA_InitStructure->DMA_PeripheralDataSize = DMA_PeripheralDataSize;
    DMA_InitStructure->DMA_MemoryDataSize = DMA_MemoryDataSize;
    DMA_InitStructure->DMA_Mode = DMA_Mode;
    DMA_InitStructure->DMA_Priority = DMA_Priority;
    DMA_InitStructure->DMA_M2M = DMA_M2M;
    DMA_Init(DMA_Channel, DMA_InitStructure);

}

void setADC_Structure(ADC_InitTypeDef* ADC_InitStructure, ADC_TypeDef* ADCx, u32 ADC_Mode, FunctionalState ADC_ScanConvMode, FunctionalState ADC_ContinuousConvMode, u32 ADC_ExternalTrigConv, u32 ADC_DataAlign, u8 ADC_NbrOfChannel)
{
    ADC_InitStructure->ADC_Mode = ADC_Mode;
    ADC_InitStructure->ADC_ScanConvMode = ADC_ScanConvMode;
    ADC_InitStructure->ADC_ContinuousConvMode = ADC_ContinuousConvMode;
    ADC_InitStructure->ADC_ExternalTrigConv = ADC_ExternalTrigConv;
    ADC_InitStructure->ADC_DataAlign = ADC_DataAlign;
    ADC_InitStructure->ADC_NbrOfChannel = ADC_NbrOfChannel;

    ADC_Init(ADCx, ADC_InitStructure);

}

void Set(GPIO_TypeDef* GPIOx, u16 GPIOPin)
{
    GPIO_SetBits(GPIOx, GPIOPin);
}

void Reset(GPIO_TypeDef* GPIOx, u16 GPIOPin)
{
    GPIO_ResetBits(GPIOx, GPIOPin);
}

void CLK_init(void)
{
    // 使能AFIO时钟和GPIO时钟
    RCC_APB2PeriphClockCmd(
        RCC_APB2Periph_AFIO |
        RCC_APB2Periph_GPIOB |
        RCC_APB2Periph_GPIOA,
        ENABLE
    );
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
}

void TIM1_Init(void)
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);

    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_OCInitTypeDef TIM_OCInitStructure;

    TIM_TimeBaseStructure.TIM_Prescaler = 36000 - 1;  // 假设APB2时钟为72MHz，预分频7200得到10kHz
    TIM_TimeBaseStructure.TIM_Period = 1000 - 1;     // 10kHz的计数频率，计数到5000得到500ms周期
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);

    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = 499;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
    TIM_OC1Init(TIM1, &TIM_OCInitStructure);

    TIM_Cmd(TIM1, ENABLE);
    TIM_CtrlPWMOutputs(TIM1, ENABLE);

    printf("TIM1 initialized.\r\n");

}

void DMA1_CH1_Init(void)
{
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

    DMA_InitTypeDef DMA_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    // 复位DMA通道
    DMA_DeInit(DMA1_Channel1);

    setDMA_Structure(
        &DMA_InitStructure,
        DMA1_Channel1,
        ADC1_DR_Address,
        (u32)ADC_RegularConvertedValueTab,
        DMA_DIR_PeripheralSRC,
        ADC1_CONV_SIZE,
        DMA_PeripheralInc_Disable,
        DMA_MemoryInc_Enable,
        DMA_PeripheralDataSize_HalfWord,
        DMA_MemoryDataSize_HalfWord,
        DMA_Mode_Normal,
        DMA_Priority_High,
        DMA_M2M_Disable
    );

    // 配置DMA1通道1

    DMA_ClearITPendingBit(DMA1_IT_TC1);
    DMA_ITConfig(DMA1_Channel1, DMA_IT_TC, ENABLE);

    setNVIC_Structure(
        &NVIC_InitStructure,
        DMA1_Channel1_IRQn,
        0,
        0,
        ENABLE
    );
    // 启动DMA
    DMA_Cmd(DMA1_Channel1, ENABLE);

}

void ADC1_Init(void)
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
    ADC_DeInit(ADC1);

    TIM1_Init();
    DMA1_CH1_Init();

    ADC_InitTypeDef ADC_InitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;

    setGPIO_Structure(
        &GPIO_InitStructure,
        B,
        GPIO_Mode_AIN,
        Pin0,
        GPIO_Speed_50MHz
    );

    setADC_Structure(
        &ADC_InitStructure,
        ADC1,
        ADC_Mode_Independent,
        DISABLE,
        DISABLE,
        ADC_ExternalTrigConv_T1_CC1,
        ADC_DataAlign_Right,
        1
    );

    //设置ADC分频因子 72MHz/6=12,ADC时钟最好不要超过14MHz
    RCC_ADCCLKConfig(RCC_PCLK2_Div6);
    //规则组输入通道配置
    ADC_RegularChannelConfig(ADC1, ADC_Channel_16, 1, ADC_SampleTime_71Cycles5);

    // 开启内部温度传感器
    ADC_TempSensorVrefintCmd(ENABLE);
    //使能ADC1的DMA
    ADC_DMACmd(ADC1, ENABLE);
    //使能ADC1
    ADC_Cmd(ADC1, ENABLE);

    // ADC1校准
    ADC_ResetCalibration(ADC1);
    while (ADC_GetResetCalibrationStatus(ADC1));

    ADC_StartCalibration(ADC1);
    while (ADC_GetCalibrationStatus(ADC1));

    // 开启外部触发转换
    ADC_ExternalTrigConvCmd(ADC1, ENABLE);

    printf("ADC1 Calibrated.\r\n");

}

void NVICc_Init(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);

    setNVIC_Structure(
        &NVIC_InitStructure,
        ADC1_2_IRQn,
        1,
        0,
        ENABLE
    );
    setNVIC_Structure(
        &NVIC_InitStructure,
        TIM1_UP_IRQn,
        1,
        2,
        ENABLE
    );
    setNVIC_Structure(
        &NVIC_InitStructure,
        TIM1_CC_IRQn,
        1,
        1,
        ENABLE
    );

}

float trans_Temperature(float voltage)
{
    float V25 = 1.43;
    float Avg_Slope = 4.3e-2;
    return (V25 - voltage) / Avg_Slope + 25;
}

float get_adc_temperature(void)
{
    // 开启软件转换
    ADC_SoftwareStartConvCmd(ADC1, ENABLE);
    while (!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC));
    // adc值
    u16 adc_value = ADC_GetConversionValue(ADC1);
    // 电压
    float voltage = (float)adc_value * (3.3 / 4096);
    // 温度
    return trans_Temperature(voltage);
}

void DMA1_Channel1_IRQHandler(void)
{
    if (DMA_GetFlagStatus(DMA1_IT_TC1) != RESET)
    {
        printf("DMA1_Channel1_IRQHandler\r\n");
        for (int i = 0; i < ADC1_CONV_SIZE; i++)
        {
            printf("No.%d: %d\r\n", i + 1, ADC_RegularConvertedValueTab[i]);
            float temp = ADC_RegularConvertedValueTab[i] * (3.3 / 4096);
            printf("\t%f\r\n", trans_Temperature(temp));
        }
        printf("Funny\r\n");
        DMA_ClearFlag(DMA1_FLAG_TC1);
        TIM_Cmd(TIM1, DISABLE);
        DMA_ClearITPendingBit(DMA1_IT_TC1);
    }
}
