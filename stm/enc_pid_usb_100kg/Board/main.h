#ifndef MAIN_H
#define MAIN_H
#include "stm32f4xx.h"

#define ENCLA_PIN               GPIO_Pin_6
#define ENCLA_GPIO_PORT         GPIOA
#define ENCLA_GPIO_CLK          RCC_AHB1Periph_GPIOA
#define ENCLA_SOURCE            GPIO_PinSource6
#define ENCLA_AF                GPIO_AF_TIM3

#define ENCLB_PIN               GPIO_Pin_7
#define ENCLB_GPIO_PORT         GPIOA
#define ENCLB_GPIO_CLK          RCC_AHB1Periph_GPIOA
#define ENCLB_SOURCE            GPIO_PinSource7
#define ENCLB_AF                GPIO_AF_TIM3

// Right Motor Channels
#define ENCRA_PIN               GPIO_Pin_9
#define ENCRA_GPIO_PORT         GPIOE
#define ENCRA_GPIO_CLK          RCC_AHB1Periph_GPIOE
#define ENCRA_SOURCE            GPIO_PinSource9
#define ENCRA_AF                GPIO_AF_TIM1

#define ENCRB_PIN               GPIO_Pin_11
#define ENCRB_GPIO_PORT         GPIOE
#define ENCRB_GPIO_CLK          RCC_AHB1Periph_GPIOE
#define ENCRB_SOURCE            GPIO_PinSource11
#define ENCRB_AF                GPIO_AF_TIM1

// determine the timers to use
#define ENCL_TIMER              TIM3
#define ENCL_TIMER_CLK          RCC_APB1Periph_TIM3
#define ENCR_TIMER              TIM1
#define ENCR_TIMER_CLK          RCC_APB2Periph_TIM1

#define LEFT_COUNT()            ENCL_TIMER->CNT
#define RIGHT_COUNT()           ENCR_TIMER->CNT



#endif


