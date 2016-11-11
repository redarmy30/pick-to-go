#include "TIM.h"


void TIM2_Configuration(void)
{
  TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;

  /* Time base configuration */
  TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
  TIM_TimeBaseStructure.TIM_Period = (84000000 / 200000) - 1; // 200 KHz, from 84 MHz TIM2CLK (ie APB1 = HCLK/4, TIM2CLK = HCLK/2)
  TIM_TimeBaseStructure.TIM_Prescaler = 0;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

  /* TIM2 TRGO selection */
  TIM_SelectOutputTrigger(TIM2, TIM_TRGOSource_Update); // ADC_ExternalTrigConv_T2_TRGO

  /* TIM2 enable counter */
  TIM_Cmd(TIM2, ENABLE);
}
