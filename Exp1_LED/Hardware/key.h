#ifndef __KEY_H__
#define __KEY_H__
#include "stm32f10x.h"

void Key_Init(void);
u8 Key_GetNum(void);
u8 Key_GetNumPro(GPIO_TypeDef *GPIOx, u16 GPIO_Pin);

#endif // !__KEY
