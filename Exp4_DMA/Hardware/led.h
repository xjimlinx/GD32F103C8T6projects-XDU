#ifndef __LED_H
#define __LED_H
#include "stm32f10x.h"

void LEDA_Init(void);
void TestPre(void);
void HolyShit(void);
void streamLight(GPIO_TypeDef* GPIOx, u16 GPIOPin, int time);
void streamLightOneTest(void);
void SetGSide(void);
void SetBSide(void);
void ResetGSide(void);
void ResetBSide(void);

void DSideLight(void);
void Turn_LED(GPIO_TypeDef *GPIOx, u16 GPIO_Pin);

#endif // !__LED_H
