#include "encoders.h"
#include "main.h"

void timEncoderConfigure(TIM_TypeDef * TIM)
{
  TIM->PSC   = 0x0;
  TIM->ARR   = 0xFFFF;
  TIM->CR1   = TIM_CR1_ARPE     | TIM_CR1_CEN;
  TIM->SMCR  = TIM_SMCR_SMS_0   | TIM_SMCR_SMS_1;
  TIM->CCMR1 = TIM_CCMR1_CC1S_0 | TIM_CCMR1_CC2S_0;
  TIM->CNT   = 0;
}

void encodersInit1 (void){
  GPIO_InitTypeDef GPIO_InitStructure;
  // turn on the clocks for each of the ports needed
  RCC_AHB1PeriphClockCmd (ENCLA_GPIO_CLK, ENABLE);
  RCC_AHB1PeriphClockCmd (ENCLB_GPIO_CLK, ENABLE);
  RCC_AHB1PeriphClockCmd (ENCRA_GPIO_CLK, ENABLE);
  RCC_AHB1PeriphClockCmd (ENCRB_GPIO_CLK, ENABLE);

  // now configure the pins themselves
  // they are all going to be inputs with pullups
  GPIO_StructInit (&GPIO_InitStructure);
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_InitStructure.GPIO_Pin = ENCLA_PIN;
  GPIO_Init (ENCLA_GPIO_PORT, &GPIO_InitStructure);
  GPIO_InitStructure.GPIO_Pin = ENCLB_PIN;
  GPIO_Init (ENCLB_GPIO_PORT, &GPIO_InitStructure);
  GPIO_InitStructure.GPIO_Pin = ENCRA_PIN;
  GPIO_Init (ENCRA_GPIO_PORT, &GPIO_InitStructure);
  GPIO_InitStructure.GPIO_Pin = ENCRB_PIN;
  GPIO_Init (ENCRB_GPIO_PORT, &GPIO_InitStructure);

  // Connect the pins to their Alternate Functions
  GPIO_PinAFConfig (ENCLA_GPIO_PORT, ENCLA_SOURCE, ENCLA_AF);
  GPIO_PinAFConfig (ENCLB_GPIO_PORT, ENCLB_SOURCE, ENCLB_AF);
  GPIO_PinAFConfig (ENCRA_GPIO_PORT, ENCRA_SOURCE, ENCRA_AF);
  GPIO_PinAFConfig (ENCRB_GPIO_PORT, ENCRB_SOURCE, ENCRB_AF);

  // Timer peripheral clock enable
  RCC_APB1PeriphClockCmd (ENCL_TIMER_CLK, ENABLE);
  RCC_APB2PeriphClockCmd (ENCR_TIMER_CLK, ENABLE);

  // set them up as encoder inputs
  // set both inputs to rising polarity to let it use both edges
  TIM_EncoderInterfaceConfig (ENCL_TIMER, TIM_EncoderMode_TI12,
                              TIM_ICPolarity_Rising,
                              TIM_ICPolarity_Rising);
  TIM_SetAutoreload (ENCL_TIMER, 0xffff);
  TIM_EncoderInterfaceConfig (ENCR_TIMER, TIM_EncoderMode_TI1,
                              TIM_ICPolarity_Rising,
                              TIM_ICPolarity_Rising);
  TIM_SetAutoreload (ENCR_TIMER, 0xffff);

  // turn on the timer/counters
  TIM_Cmd (ENCL_TIMER, ENABLE);
  TIM_Cmd (ENCR_TIMER, ENABLE);
  encodersReset();
}

void encodersInit (void){
GPIO_InitTypeDef GPIO_InitStructure;
RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3|RCC_APB2Periph_TIM1, ENABLE);

GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7 ;
GPIO_Init(GPIOA, &GPIO_InitStructure);
GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9|GPIO_Pin_11;
GPIO_Init(GPIOE, &GPIO_InitStructure);

GPIO_PinAFConfig(GPIOA, GPIO_PinSource6, GPIO_AF_TIM3);
GPIO_PinAFConfig(GPIOA, GPIO_PinSource7, GPIO_AF_TIM3);
//GPIO_PinAFConfig(GPIOE, GPIO_PinSource9, GPIO_AF_TIM1);
//GPIO_PinAFConfig(GPIOE, GPIO_PinSource11, GPIO_AF_TIM1);

TIM_SetAutoreload (TIM3, 0xffffffff);
TIM_SetAutoreload (TIM1, 0xffffffff);
/* Configure the timer */
//TIM_EncoderInterfaceConfig(TIM1, TIM_EncoderMode_TI1, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);
TIM_EncoderInterfaceConfig(TIM3, TIM_EncoderMode_TI1, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);
/* TIM2/5 counter enable */

//TIM_Cmd(TIM1, ENABLE);
TIM_Cmd(TIM3, ENABLE);
  conf_pin(pin_id(PORTE,9), 2, PUSH_PULL, 0, PULL_UP);   //Ёнкодер 4 A
  conf_af(pin_id(PORTE,9), AF1);
  conf_pin(pin_id(PORTE,11), 2, PUSH_PULL, 0, PULL_UP);   //Ёнкодер 4 B
  conf_af(pin_id(PORTE,11), AF1);
  timEncoderConfigure(TIM1);
  encodersReset();
}


void encodersReset (void)
{
  __disable_irq();
  oldLeftEncoder = 0;
  oldRightEncoder = 0;
  leftTotal = 0;
  rightTotal = 0;
  fwdTotal = 0;
  rotTotal = 0;
  TIM_SetCounter (TIM1, 0);
  TIM_SetCounter (TIM3, 0);
  encodersRead();
  __enable_irq();
}

//available to the rest of the code
//speeds


void encodersRead (void)
{
  oldLeftEncoder = leftEncoder;
  leftEncoder = TIM_GetCounter (TIM1) ;
  oldRightEncoder = rightEncoder;
  rightEncoder = -TIM_GetCounter (TIM3) ;
  leftCount = leftEncoder - oldLeftEncoder;
  rightCount = rightEncoder - oldRightEncoder;
  fwdCount = leftCount + rightCount;
  rotCount = - (leftCount - rightCount);
  fwdTotal += fwdCount;
  rotTotal += rotCount;
  leftTotal += leftCount;
  rightTotal += rightCount;
}







 /* conf_pin(pin_id(PORTE,9), 2, PUSH_PULL, 0, PULL_UP);   //Ёнкодер 4 A
  conf_af(ENCODER4A_PIN, AF1);
  conf_pin(pin_id(PORTE,11), 2, PUSH_PULL, 0, PULL_UP);   //Ёнкодер 4 B
  conf_af(ENCODER4B_PIN, AF1);
*/


