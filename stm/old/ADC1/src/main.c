#include <stm32f4xx.h>
#include <misc.h>			 // I recommend you have a look at these in the ST firmware folder
#include <stm32f4xx_usart.h> // under Libraries/STM32F4xx_StdPeriph_Driver/inc and src
/*
#define MAX_STRLEN 12 // this is the maximum string length of our string in characters

#define GENERAL_PIN_0            pin_id(PORTA,0)                //ADC12_IN0//
#define GENERAL_PIN_1            pin_id(PORTA,1)                //ADC12_IN1//
#define GENERAL_PIN_2            pin_id(PORTA,2)                //ADC12_IN2//
#define GENERAL_PIN_3            pin_id(PORTA,3)                //ADC12_IN3//
#define GENERAL_PIN_4            pin_id(PORTB,0)                //ADC12_IN8//
#define GENERAL_PIN_5            pin_id(PORTB,1)                //ADC12_IN9//
#define GENERAL_PIN_6            pin_id(PORTC,1)                //ADC12_IN11//
#define GENERAL_PIN_7            pin_id(PORTC,2)                //ADC12_IN12//
#define GENERAL_PIN_8            pin_id(PORTC,4)                //ADC12_IN14//
#define GENERAL_PIN_9            pin_id(PORTC,5)                //ADC12_IN15//
#include "Interrupts.h"
#include "gpio.h" //
#include "ADC.h"
//extern uint16_t adcData[10];


int main(void)
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1,  ENABLE); // ADC1
    conf_pin(GENERAL_PIN_0, INPUT, PUSH_PULL, FAST_S, PULL_DOWN);//conf_pin(GENERAL_PIN_0, ANALOG, PUSH_PULL, FAST_S, NO_PULL_UP);
    conf_pin(GENERAL_PIN_1, ANALOG, PUSH_PULL, FAST_S, NO_PULL_UP);
    conf_pin(GENERAL_PIN_2, ANALOG, PUSH_PULL, FAST_S, NO_PULL_UP);
    conf_pin(GENERAL_PIN_3, ANALOG, PUSH_PULL, FAST_S, NO_PULL_UP);
    conf_pin(GENERAL_PIN_4, ANALOG, PUSH_PULL, FAST_S, NO_PULL_UP);
    conf_pin(GENERAL_PIN_5, ANALOG, PUSH_PULL, FAST_S, NO_PULL_UP);
    conf_pin(GENERAL_PIN_6, ANALOG, PUSH_PULL, FAST_S, NO_PULL_UP);
    conf_pin(GENERAL_PIN_7, ANALOG, PUSH_PULL, FAST_S, NO_PULL_UP);
    conf_pin(GENERAL_PIN_8, ANALOG, PUSH_PULL, FAST_S, NO_PULL_UP);
    conf_pin(GENERAL_PIN_9, ANALOG, PUSH_PULL, FAST_S, NO_PULL_UP);
  adcConfig();

  TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

	// базовая нстйрока
	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
	TIM_TimeBaseStructure.TIM_Period = 625 - 1;
	TIM_TimeBaseStructure.TIM_Prescaler = 2000 - 1;
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV4;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

	// выход синхронизации
	TIM_SelectOutputTrigger(TIM3, TIM_TRGOSource_Update);

	// запуск таймера
	TIM_Cmd(TIM3, ENABLE);
uint16_t test;
  while (1)
  {
//      test = adcData[1];
  }
  }

*/
void DMA2_Stream4_IRQHandler(void)
{
      GPIOD->ODR        ^=((1<<12)|(1<<13)|(1<<14)|(1<LIFCR |=DMA_LIFCR_CTCIF4));
}

uint16_t BUFF[100];
int main(void)
{
    SystemInit();

//*********************GPIO*************************
        RCC->AHB1ENR    |= RCC_AHB1ENR_GPIOAEN;
        RCC->AHB1ENR    |= RCC_AHB1ENR_GPIODEN;
        GPIOA->MODER    |= GPIO_MODER_MODER6;        //ADC1 CH6 PA6


//*********************TIM3*************************
        RCC->APB1ENR    |= RCC_APB1ENR_TIM3EN;        // тактирование таймера
        TIM3->PSC         = 160-1;                    // предделитель
        TIM3->ARR         = 1000-1;                    // переполнение
        TIM3->CR2         |= TIM_CR2_MMS_1;            // output (TRGO)
        TIM3->DIER         |= TIM_DIER_UDE;
        TIM3->CR1         |= TIM_CR1_CEN;                //запуск счета


//********************DMA***************************
        RCC->AHB1ENR    |= RCC_AHB1ENR_DMA2EN;
        DMA2_Stream4->CR&=~ DMA_SxCR_CHSEL;            // 000: channel 0 selected
        DMA2_Stream4->PAR= (uint32_t)&ADC1->DR;     //
        DMA2_Stream4->M0AR=(uint32_t)&BUFF[0];        // Массив
        DMA2_Stream4->NDTR=100;                        // Длина буфера
        DMA2_Stream4->CR|= DMA_SxCR_MINC;            // Инкремент адреса
        DMA2_Stream4->CR|= DMA_SxCR_MSIZE_0;        // 16 бит
        DMA2_Stream4->CR|= DMA_SxCR_PSIZE_0;        // 16 бит
        DMA2_Stream4->CR|= DMA_SxCR_CIRC;            // Непрерывный режим
        DMA2_Stream4->CR&=~ DMA_SxCR_DIR;            // 01: peripheral-to-Memory
        DMA2_Stream4->CR|= DMA_SxCR_PL;                // 11: Very high приоритет
        DMA2_Stream4->CR|= DMA_SxCR_TCIE;            // Transfer complete interrupt enable
        DMA2_Stream4->CR|= DMA_SxCR_EN;                 // Вкл. передачу
        NVIC_EnableIRQ(DMA2_Stream4_IRQn);
        NVIC_SetPriority(DMA2_Stream4_IRQn,5);


//********************ADC1**CH6 PA6***************
        RCC->APB2ENR    |= RCC_APB2ENR_ADC1EN;
        ADC1->CR1         |=ADC_CR1_SCAN;                // Scan mode
        ADC1->CR2        |=(ADC_CR2_EXTSEL_3);        //tim 3 (TRGO)
        ADC1->SQR3        =6;                            //выбор канала PA6
        ADC1->CR2         |=ADC_CR2_DMA;                //разрешаем рабуту DMA
        ADC1->CR2         |=ADC_CR2_DDS;                //DMA disable selection (for single ADC mode)
        ADC1->CR2        &=~ADC_CR2_CONT;            //Continuous conversion
        ADC1->CR2        |=ADC_CR2_EXTEN_0;            //01: Trigger detection on the rising edge
        ADC1->CR2        |= ADC_CR2_EOCS;            //разрешаем прерывания
        ADC1->CR2         |=ADC_CR2_ADON;                //Вкл. переобразования
}
