#ifndef MAIN_H
#define MAIN_H
#include "stm32f4xx.h"

#define ENCLA_PIN               GPIO_Pin_15
#define ENCLA_GPIO_PORT         GPIOA
#define ENCLA_GPIO_CLK          RCC_AHB1Periph_GPIOA
#define ENCLA_SOURCE            GPIO_PinSource15
#define ENCLA_AF                GPIO_AF_TIM2

#define ENCLB_PIN               GPIO_Pin_3
#define ENCLB_GPIO_PORT         GPIOB
#define ENCLB_GPIO_CLK          RCC_AHB1Periph_GPIOB
#define ENCLB_SOURCE            GPIO_PinSource3
#define ENCLB_AF                GPIO_AF_TIM2

// Right Motor Channels
#define ENCRA_PIN               GPIO_Pin_8
#define ENCRA_GPIO_PORT         GPIOB
#define ENCRA_GPIO_CLK          RCC_AHB1Periph_GPIOB
#define ENCRA_SOURCE            GPIO_PinSource6
#define ENCRA_AF                GPIO_AF_TIM4

#define ENCRB_PIN               GPIO_Pin_9
#define ENCRB_GPIO_PORT         GPIOB
#define ENCRB_GPIO_CLK          RCC_AHB1Periph_GPIOB
#define ENCRB_SOURCE            GPIO_PinSource7
#define ENCRB_AF                GPIO_AF_TIM4

// determine the timers to use
#define ENCL_TIMER              TIM2
#define ENCL_TIMER_CLK          RCC_APB1Periph_TIM2
#define ENCR_TIMER              TIM4
#define ENCR_TIMER_CLK          RCC_APB1Periph_TIM4

#define LEFT_COUNT()            ENCL_TIMER->CNT
#define RIGHT_COUNT()           ENCR_TIMER->CNT


#endif


