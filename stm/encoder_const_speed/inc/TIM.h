
#ifndef TIM_H
#define TIM_H










#include "stm32f4xx.h"
#define TIM6_TIME  0.0025// (84000000/840/250)^-1
void TIM2_Configuration(void);
void TIM6_Config(void);
void TIM7_Config(void);



#endif
