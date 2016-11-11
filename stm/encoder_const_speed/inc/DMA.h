

#ifndef DMA_H
#define DMA_H

#include "stm32f4xx.h"

#define BUFFERSIZE 2 // 200KHz x2 HT/TC at 1KHz
__IO uint16_t ADCConvertedValues[BUFFERSIZE];

static void DMA_Configuration(void);
static void DMA_Configuratio_new(void);

#endif
