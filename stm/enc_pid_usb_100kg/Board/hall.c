
#include "hall.h"


void Inithall(void){

    // initialize 3 hall sensors at pins a0,a1,a2 on tim2 with the interrupt TIM2_IRQn
  GPIO_InitTypeDef GPIO_InitStructure;
  TIM_TimeBaseInitTypeDef TIM_InitStructure;
  TIM_ICInitTypeDef TIM_ICInitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;

  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;

  GPIO_Init(GPIOC, &GPIO_InitStructure);

  // enable clock for GPIOA
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

  // inputs for hall sensors
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  //GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  //GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  //GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;

  GPIO_Init(GPIOA, &GPIO_InitStructure);

  GPIO_PinAFConfig(GPIOA, GPIO_PinSource0, GPIO_AF_TIM2);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource1, GPIO_AF_TIM2);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_TIM2);

  // enable clock for timer2
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

  // update event every ca. 4295sec
  // resolution: 1usec // 126 => 3,5s till overflow ; 285,714kHz TimerClock [36MHz/Prescaler]
  TIM_InitStructure.TIM_Prescaler = 126;
  TIM_InitStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_InitStructure.TIM_Period = 65535;
    TIM_InitStructure.TIM_ClockDivision = 0;
  TIM_InitStructure.TIM_RepetitionCounter = 0;

  TIM_TimeBaseInit(TIM2, &TIM_InitStructure);
  // connect three channels to XOR in timer
  TIM_SelectHallSensor(TIM2, ENABLE);

  TIM_SelectInputTrigger(TIM2, TIM_TS_TI1F_ED);
  TIM_SelectSlaveMode(TIM2, TIM_SlaveMode_Reset);

  // initialize the cature compare function of timer2
  TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;
  TIM_ICInitStructure.TIM_ICFilter = 0xF;
  TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_BothEdge;
  TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
  TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_TRC;

  TIM_ICInit(TIM2, &TIM_ICInitStructure);

   // enable the interrupt for timer2
  TIM_ITConfig(TIM2, TIM_IT_CC1, ENABLE);

  NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

   // enable timer2
  TIM_Cmd(TIM2, ENABLE);
}
