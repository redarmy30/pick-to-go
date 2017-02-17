#ifndef INTERA_H
#define INTERA_H
#include "stm32f4xx.h"
#include "PID.h"







#define WHEELDIAM 0.19
#define TICKSPERROTATION 3400
#define PI  3.1415926535897932384626433832795
#define DISTANCEBTWWHEELS 0.73

void USART2_IRQHandler(void);
void USART6_IRQHandler(void);
void DMA2_Stream0_IRQHandler(void);
void TIM6_DAC_IRQHandler(void);
void TIM7_IRQHandler(void);
void TIM2_IRQHandler(void);
void rotateMe (float  *angle);
void GoForward(float *point);
void checkposition(void);

#endif
