#include <stm32f4xx.h>
#include <misc.h>			 // I recommend you have a look at these in the ST firmware folder
#include <stm32f4xx_usart.h> // under Libraries/STM32F4xx_StdPeriph_Driver/inc and src
#include <stdio.h>
#include <stdlib.h>
#include "USART.h"


#include "main.h"
#include "encoders.h"
#include "DMA.h"
#include "TIM.h"
#include "ADC.h"
#include "USART.h"

#define MAX_STRLEN 12 // this is the maximum string length of our string in characters

#define MAX_STRLEN 12 // this is the maximum string length of our string in characters


volatile char received_string[MAX_STRLEN+1]; // this will hold the recieved string
uint32_t multiplier;
int counter=0;
int counter1=0;
int speed=0;
int speed1=0;
int s = 255;
int sp = 90;
char test;
int flag = 0;
int Last_tick=0;
int passed_tick = 0 ;
int task=0;



void TM_Delay_Init(void) {
    RCC_ClocksTypeDef RCC_Clocks;

    /* Get system clocks */
    RCC_GetClocksFreq(&RCC_Clocks);

    /* While loop takes 4 cycles */
    /* For 1 us delay, we need to divide with 4M */
    multiplier = RCC_Clocks.HCLK_Frequency / 4000000;
}

void TM_DelayMicros(uint32_t micros) {
    /* Multiply micros with multipler */
    /* Substract 10 */
    micros = micros * multiplier - 10;
    /* 4 cycles for one loop */
    while (micros--);
}

void TM_DelayMillis(uint32_t millis) {
    /* Multiply millis with multipler */
    /* Substract 10 */
    millis = 1000 * millis * multiplier - 10;
    /* 4 cycles for one loop */
    while (millis--);
}

void Delay(__IO uint32_t nCount) {
  while(nCount--) {
  }
}


//_____________________________________________________________________//
 /* Enable peripheral clocks */

void RCC_Configuration(void)
{ /*
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
  RCC_AHB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);*/
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_CRC, ENABLE);   // PORTA
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE); // PORTA
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE); // PORTB
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE); // PORTC
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE); // PORTD
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE); // PORTE

  RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1,  ENABLE); // TIM1
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,  ENABLE); // TIM2
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,  ENABLE); // TIM3
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4,  ENABLE); // TIM4
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5,  ENABLE); // TIM4
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6,  ENABLE); // TIM6
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7,  ENABLE); // TIM7
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8,  ENABLE); // TIM8
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM9,  ENABLE); // TIM9
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM10, ENABLE); // TIM10
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM11, ENABLE); // TIM11
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM12, ENABLE); // TIM12
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM13, ENABLE); // TIM13
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1,  ENABLE); // ADC1


  RCC_APB1PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE); // USART1
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3,ENABLE); // USART3

  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1,  ENABLE); // DMA1
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2,  ENABLE); // DMA2

  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG,ENABLE); // SYSCFG
}
void GPIO_Configuration(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_2 | GPIO_Pin_1;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;\
  GPIO_Init(GPIOC, &GPIO_InitStructure);
}

void NVIC_Configuration(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;

  /* Enable the DMA Stream IRQ Channel */
  NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream0_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}



int main(void) {

  RCC_Configuration();
  GPIO_Configuration();
  NVIC_Configuration();
  TIM6_Config();
  TIM7_Config();
  TIM2_Configuration();
  DMA_Configuration_new();
  ADC_Configuration_new();

  //TM_Delay_Init();
  init_USART1_1(27500*3); // initialize USART1 @ 9600 baud \\TC: Transmission complete
  init_USART6_1(27500*3);//26315
  encodersInit();

  int speeadc1 = 0;
  int speeadc2 = 0;
  int speeadc3 = 0;
  speed = -70;
  speed1 = -70;
  task = 16;
  flag = 1;
  while (1)
  {
    speeadc3 = TIM_GetCounter(TIM6);
  }
}




int k =0;
int kk =0;



void USART1_IRQHandler(void){
	// check if the USART1 receive interrupt flag was set
	if (USART_GetITStatus(USART1,USART_IT_TC))
    {
         USART_ClearITPendingBit(USART1,USART_IT_TC);
        if (counter == 0)
        {   //speed = ((ADCConvertedValues[0]-2055) / 12);
            USART_SendData(USART1,(~s)& 0xFF);
           /* if (ADCConvertedValues[0]>3500){speed = 65;speed1=-65;}
            if (ADCConvertedValues[0]<500){speed = -60;speed1=60;}
            if (ADCConvertedValues[0]>1000 & ADCConvertedValues[0]<3000){speed = 0;speed1=0;}
            if (ADCConvertedValues[1]>3700 & (ADCConvertedValues[0]>1000 & ADCConvertedValues[0]<3000)  ) {speed1+=70; speed+=70;}
            if (ADCConvertedValues[1]<600 & (ADCConvertedValues[0]>1000 & ADCConvertedValues[0]<3000 )  ) {speed1-=70;speed-=70;}
           */


        }
        if (counter == 1)
        {
            USART_SendData(USART1,(~s)& 0xFF);
        }
        if (counter == 2)
        {
            USART_SendData(USART1,256);
        }
        if (counter == 3)
        {
            USART_SendData(USART1,speed & 0xFF);
        }
        if (counter == 4)
        {
            USART_SendData(USART1,(speed >> 8) & 0xFF);
        }
        if (counter == 5)
        {
            USART_SendData(USART1,speed & 0xFF);
        }
        if (counter == 6)
        {
            USART_SendData(USART1,(speed >> 8) & 0xFF);
        }
        if (counter == 7)
        {
            USART_SendData(USART1,85);
             s--;
            if (s==0) {s=255; k++;}
            if (k ==9) k=0;
        }
                //USART_ClearITPendingBit(USART1,USART_IT_TC);
        counter++;
        if (counter>7)
        {counter=0;}
    }

}

void USART6_IRQHandler(void){

    if ( USART_GetITStatus(USART6, USART_IT_TC))
    {   USART_ClearITPendingBit(USART6,USART_IT_TC);
        if (counter1 == 0)
        {
            USART_SendData(USART6,(~sp)& 0xFF);
            //speed1 =-((ADCConvertedValues[1]-2055) / 12);

            //speed1 = ((ADCConvertedValues[1] - 2055) / 6);
        }
        if (counter1 == 1)
        {
            USART_SendData(USART6,(~sp)& 0xFF);
        }
        if (counter1 == 2)
        {
            USART_SendData(USART6,256);
        }
        if (counter1 == 3)
        {
            USART_SendData(USART6,speed1 & 0xFF);
        }
        if (counter1 == 4)
        {
            USART_SendData(USART6,(speed1 >> 8) & 0xFF);
        }
        if (counter1 == 5)
        {
            USART_SendData(USART6,speed1 & 0xFF);
        }
        if (counter1 == 6)
        {
            USART_SendData(USART6,(speed1 >> 8) & 0xFF);
        }
        if (counter1 == 7)
        {
            USART_SendData(USART6,85);

            if (sp==92) sp=90;
            if (sp==90) sp=92;
        }
        //USART_ClearITPendingBit(USART1,USART_IT_TC);
        counter1++;
        if (counter1>7)
            counter1=0;
    }

}

void DMA2_Stream0_IRQHandler(void) // Called at 1 KHz for 200 KHz sample rate, LED Toggles at 500 Hz
{
  /* Test on DMA Stream Half Transfer interrupt */
  if(DMA_GetITStatus(DMA2_Stream0, DMA_IT_HTIF0))
  {
    /* Clear DMA Stream Half Transfer interrupt pending bit */
    DMA_ClearITPendingBit(DMA2_Stream0, DMA_IT_HTIF0);

    /* Turn LED3 off: Half Transfer */


    // Add code here to process first half of buffer (ping)
  }
  /* Test on DMA Stream Transfer Complete interrupt */
  if(DMA_GetITStatus(DMA2_Stream0, DMA_IT_TCIF0))
  {
    /* Clear DMA Stream Transfer Complete interrupt pending bit */
    DMA_ClearITPendingBit(DMA2_Stream0, DMA_IT_TCIF0);
  }
}
float wheelspeed = 0;


void TIM6_DAC_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM6, TIM_IT_Update) != RESET)
    {
        TIM_ClearITPendingBit(TIM6, TIM_IT_Update);
        encodersRead();
        wheelspeed = leftCount *0.18/16;
    }
}

void TIM7_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM7, TIM_IT_Update) != RESET)
    {
        encodersRead();
        TIM_ClearITPendingBit(TIM7, TIM_IT_Update);
        if (!flag) {speed = 0;}
        else
        {
          if (passed_tick== 0)
          {
           Last_tick = leftTotal;
           passed_tick = 1;
          }
         else
             sp = abs(-leftTotal+Last_tick);
             {  if (sp < 20)
                    passed_tick += sp;
                Last_tick = leftTotal;
                speed = -200;
             }

         if (passed_tick >= task+1) {speed = 0 ; flag = 0; passed_tick=0;}
        }
    }
}
