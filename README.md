# GD32F103C8T6projects-XDU
XDU的GD32F103的单片机实验

（好吧，给的板子是**CBT6**的）

## 实验一

​        *根据原理图*

​        *PA8  -> KEY0        上拉输入*

​        *PB10 -> KEY1        上拉输入*

​        *PC13 -> LED0*

​        *PB2  -> LED1*

​        *其次设置USART接口*

​        *PA9  -> USART1_TX*

​        *PA10 -> USART1_RX*

​        *①：*

​            *按下KEY0，LED0灭，松开KEY0，LED0亮*

​            *按下KEY1，LED1灭，松开KEY1，LED1亮*

​        *②：*

​            *没有按下按键，两个LED灯都亮*

​            *按下KEY0，两个灯同步周期性闪烁*

​            *按下KEY1，两个灯反向周期性闪烁*

## 实验二

​        *根据硬件原理图配置对应的GPIO口和定时器，*

​        *实现下述功能：*

​            *配置两个LED和串口相关的IO*功能

​            *配置TIM2，根据系统72MHz时钟，*

​            *配置定时周期为1秒，向上计数模式；*

​            *然后在while循环中查询等待定时器时间到，*

​            *点亮两个LED灯，再次等待定时时间到，*

​            *熄灭两个LED灯，循环往复。*

## 实验三

​        *根据实验原理图配置对应的GPIO口和定时器，实现下述功能:*

​            *1.配置LED1、LED2和串口相关的IO功能；*

​            *2.配置TIM2，根据系统72MHz时钟，配置定时周期为1秒，向上计数模式；*

​            *3.配置TIM2的中断，在中断响应函数中翻转PC13的输出状态，实现LED1的闪烁；*

​            *4.配置PA8、PB10为上拉输入，使能AFIO；*

​            *5.配置EXTI9_5、EXTI15_10的中断优先级；*

​            *6.配置PA8、PB10的中断模式为下降沿中断；*

​            *7.在PA8的中断响应函数中点亮LED2。*

​            *8.在PB10的中断响应函数中熄灭LED2。*

## 实验四

​        *根据实验原理图配置对应的GPIO口和定时器，实现下述功能:*

​            *1.定义片内Flash上的数组和片内Mem上的数组变量；*

​            *2.配置DMA1的通道6，实现从Flash数据到Mem数据的搬运；*

​            *3.在while循环中对搬运结果进行CPU比对，*

​           	*如果Mem中的数据和Flash中的数据完全一致*

​           	*则LED2亮，否则LED2灭*

​            *4.编写PA8、PB10的中断响应函数，*

​           	*PA8对应按键按下重新启动DMA，*

​           	*PB10的按键按下时通过CPU实现Flash数据到Mem的搬运；*

​           	*5.PA8中断中启动DMA前灭灯LED1，*

​          	*DMA搬运完成的中断函数中点亮LED1；*

​            *6.根据Keil工程中初始化的TIM2，*

​           	*调用TIM_GetCounter(TIM2)获取当前的计数值；*

​           	*在PA8中断启动DMA后获取计数值以及DMA完成后获取计数值，*

​           	*可以得到DMA搬运的消耗时间；*

​           	*在PB10中断CPU搬运数据前后获取计数值，*

​           	*也可以得到CPU搬运所需的时间；*

​           	*对比输出DMA搬运和CPU搬运所需的时间。*
