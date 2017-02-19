#include <misc.h>			 // I recommend you have a look at these in the ST firmware folder
#include <stm32f4xx_usart.h> // under Libraries/STM32F4xx_StdPeriph_Driver/inc and src
#include <stdio.h>
#include <stdlib.h>
#include "main.h"
#include "encoders.h"
#include "DMA.h"
#include "TIM.h"
#include "ADC.h"

#include "PID.h"
#include "Dynamixel_control.h"
#define MAX_STRLEN 12 // this is the maximum string length of our string in characters

#define MAX_STRLEN 12 // this is the maximum string length of our string in characters



#include "stm32f4xx_conf.h"

#include "stm32f4xx.h"
#include "Board.h"  //файл инициализации

#include "gpio.h" // работа с портами ввода-вывода
#include "Pins.h" // определение ножек на плате
#include "Interrupts.h"
#include "regulator.h"  // регуляторы колес, кинематика, траекторный

#include "usart.h" //обмен с измерительной тележкой
#include "robot.h"  //определение конфигурации робота и его основных функций
#include "Manipulators.h" */
// обмен с компьютером
#include "usbd_cdc_core.h"
#include "usbd_usr.h"
#include "usb_conf.h"
#include "usbd_desc.h"
#include "usbd_cdc_vcp.h"

#ifdef USB_OTG_HS_INTERNAL_DMA_ENABLED
  #if defined ( __ICCARM__ ) /*!< IAR Compiler */
    #pragma data_alignment=4
  #endif
#endif /* USB_OTG_HS_INTERNAL_DMA_ENABLED */

__ALIGN_BEGIN USB_OTG_CORE_HANDLE    USB_OTG_dev __ALIGN_END;

char mode;
//_____________________________________________________________//
float distance[2] = {0,0};
extern robotstate telega;
float taskrot = 0;
float taskmove = 0;
int regulatorboth=0;

//float motorSpeed[2];
int pidflag = 0;
uint32_t multiplier;



/*

typedef struct
{
  float center [3];
  char (*movTask)(void);
  char (*endTask)(void);
  float endTaskP1;
  float (*speedVelTipe);
  float (*speedRotTipe);
  char step;
  float lengthTrace;
}pathPointStr;

*/

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
//_____________________________________________________________________________________________________________//
 /* Enable peripheral clocks */

void RCC_Configuration(void)
{
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


  RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE); // USART1
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3,ENABLE); // USART3
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE); // USART3


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

//_____________________________________________________________________________________//
int main(void)
{
    __disable_irq();

      USBD_Init(&USB_OTG_dev,
#ifdef USE_USB_OTG_HS
            USB_OTG_HS_CORE_ID,
#else
            USB_OTG_FS_CORE_ID,
#endif
            &USR_desc,
            &USBD_CDC_cb,
            &USR_cb);

  initRegulators1();
  RCC_Configuration();
  GPIO_Configuration();
  NVIC_Configuration();
  TIM6_Config();
  TIM7_Config();
  //TIM2_Configuration();
  Inithall();
  DMA_Configuration_new();
  ADC_Configuration_new();
  //TM_Delay_Init();
  init_USART2_1(27500); // initialize USART1 @ 9600 baud \\TC: Transmission complete
  init_USART6_1(27500);//26315
  encodersInit();

__enable_irq();
float r=5.0, angle = 90, lr =1;

  while (1)
  {
    {rotateMe(&taskrot);}
    {GoForward(&taskmove);}


    //char speed[2];

    telega.speed[0]=telega.speed[0];

    //speed[1] =(char)telega.rightwheel.speed;
    //if (regulatorboth!=0)
    //{
      //      regulatorOut[0]=regulatorboth;
        //    regulatorOut[1]=regulatorboth;
    //}
  }
}

