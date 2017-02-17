

#ifndef DMA1_H
#define DMA1_H

#include "stm32f4xx.h"

#define BUFFERSIZE 2 // 200KHz x2 HT/TC at 1KHz
__IO uint16_t ADCConvertedValues[BUFFERSIZE];

 void DMA_Configuration(void);
void DMA_Configuration_new(void);

#endif
