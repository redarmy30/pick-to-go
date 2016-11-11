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


 /*
   * definitions for the quadrature encoder pins
   */
// Left Motor Channels




volatile char received_string[MAX_STRLEN+1]; // this will hold the recieved string
uint32_t multiplier;
int counter=0;
int counter1=0;
int speed=200;
int speed1=200;
int s = 255;
int sp = 90;
char test;


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
{
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
}
void GPIO_Configuration(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;

  /* ADC Channel 11 -> PC1
  */

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
  encodersInit();
  RCC_Configuration();
  GPIO_Configuration();
  NVIC_Configuration();
  TIM2_Configuration();
  DMA_Configuratio_new();
  ADC_Configuration_new();
  TM_Delay_Init();
  init_USART1_1(27500*3); // initialize USART1 @ 9600 baud \\TC: Transmission complete
  init_USART6_1(27500*3);//26315
  int speeadc1 = 0;
  int speeadc2 = 0;
  int speeadc3 = 0;


  while (1)
  {
    encodersRead();
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
            speed = 0;
            speed1 = 0;

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
