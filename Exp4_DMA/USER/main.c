#include "stm32f10x.h"
#include "customdef.h"
#include "key.h"

/*
    实验四
    内容：
        根据实验原理图配置对应的GPIO口和定时器，实现下述功能:
            1.定义片内Flash上的数组和片内Mem上的数组变量；
            2.配置DMA1的通道6，实现从Flash数据到Mem数据的搬运；
            3.在while循环中对搬运结果进行CPU比对，
            如果Mem中的数据和Flash中的数据完全一致
            则LED2亮，否则LED2灭
            4.编写PA8、PB10的中断响应函数，
            PA8对应按键按下重新启动DMA，
            PB10的按键按下时通过CPU实现Flash数据到Mem的搬运；
            5.PA8中断中启动DMA前灭灯LED1，
            DMA搬运完成的中断函数中点亮LED1；
            6.根据Keil工程中初始化的TIM2，
            调用TIM_GetCounter(TIM2)获取当前的计数值；
            在PA8中断启动DMA后获取计数值以及DMA完成后获取计数值，
            可以得到DMA搬运的消耗时间；
            在PB10中断CPU搬运数据前后获取计数值，
            也可以得到CPU搬运所需的时间；
            对比输出DMA搬运和CPU搬运所需的时间。

*/

#define BufferSize 32

__IO u32 CurrDataCounterBegin = 0;
__IO u32 CurrDataCounterEnd = 0x01;

u32 SRC_Const_Buffer[BufferSize] = {
    0x01020304, 0x05060708, 0x090A0B0C, 0x0D0E0F10,
    0x11121314, 0x15161718, 0x191A1B1C, 0x1D1E1F20,
    0x21222324, 0x25262728, 0x292A2B2C, 0x2D2E2F30,
    0x31323334, 0x35363738, 0x393A3B3C, 0x3D3E3F40,
    0x41424344, 0x45464748, 0x494A4B4C, 0x4D4E4F50,
    0x51525354, 0x55565758, 0x595A5B5C, 0x5D5E5F60,
    0x61626364, 0x65666768, 0x696A6B6C, 0x6D6E6F70,
    0x71727374, 0x75767778, 0x797A7B7C, 0x7D7E7F80 };
u32 DST_Buffer[BufferSize];

int main(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;

    TestStatus TransferStatus = FAILED;

    // 启用GPIOA、GPIOB、GPIOC时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);

    // 启用USART1时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);

    // 启用PC13、PB2
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

    // 初始化LED灯状态为灭
    // 因为使用的是上接VCC，所以低电平亮，高电平灭
    GPIO_SetBits(C, Pin13);
    GPIO_SetBits(B, Pin2);

    // 使能各个中断
    INT_init();

    while (1)
    {
        // 通过变量判断DMA是否完成
        while (CurrDataCounterEnd != 0)
        {
        }

        // 比较两个数组是否相等
        TransferStatus = Buffercmp(SRC_Const_Buffer, DST_Buffer, BufferSize);

        if (TransferStatus == PASSED)
        {
            // PB2亮
            GPIO_ResetBits(B, Pin2);
        }
        else
        {
            // PB2灭
            GPIO_SetBits(B, Pin2);
        }

        CurrDataCounterEnd = 0x01;
    }
}
