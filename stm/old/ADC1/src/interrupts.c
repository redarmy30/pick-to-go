#include "Interrupts.h"

#include "gpio.h"



#include <math.h>





int indexSpeeds = 0, indexDists = 0;
char traceFlag, movFlag, endFlag;

int16_t int_cnt = 0;



////////////////////////////////////////////////////////////////////////////////
//_________________________________TIMERS_____________________________________//
////////////////////////////////////////////////////////////////////////////////
void TIM2_IRQHandler(void)
{
  //USB_OTG_BSP_TimerIRQ();
}
////////////////////////////////////////////////////////////////////////////////

void TIM6_DAC_IRQHandler() // 100Hz  // Рассчет ПИД регуляторов колес
{
//static char i=0; // Divider by 2 to get 10Hz frequency
   //   set_pin(PWM_DIR[8]);


  TIM6->SR = 0;

  NVIC_DisableIRQ(TIM8_UP_TIM13_IRQn);


}
////////////////////////////////////////////////////////////////////////////////

void TIM7_IRQHandler() // 33kHz
{
  TIM7->SR = 0;
////////////////////////////////////////////////////////////////////////////////
}
////////////////////////////////////////////////////////////////////////////////


void TIM8_UP_TIM13_IRQHandler() // рассчет траекторного регулятора
{
   // set_pin(PWM_DIR[8]);

  TIM13->SR = 0;
 NVIC_DisableIRQ(TIM6_DAC_IRQn);  //отключение ПИД на время расчета




    ////////////////////////////////////////////////////////////////////////////////
  NVIC_EnableIRQ(TIM6_DAC_IRQn); //включение ПИД
    // reset_pin(PWM_DIR[8]);
}



////////////////////////////////////////////////////////////////////////////////
//__________________________________EXTI______________________________________//
////////////////////////////////////////////////////////////////////////////////


//#define EXTI2_PIN               pin_id(PORTD,0)         //Разъем EXTI2//
void EXTI0_IRQHandler(void)
{

}

//#define EXTI5_PIN               pin_id(PORTD,1)         //Разъем EXTI5//
void EXTI1_IRQHandler(void)
{

}

//#define EXTI4_PIN               pin_id(PORTD,2)         //Разъем EXTI4//
void EXTI2_IRQHandler(void)
{

}

//#define EXTI6_PIN               pin_id(PORTD,3)         //Разъем EXTI6//
void EXTI3_IRQHandler(void)
{


}

//#define EXTI9_PIN               pin_id(PORTE,4)         //Разъем EXTI9//
void EXTI4_IRQHandler(void)
{


}

//#define EXTI7_PIN               pin_id(PORTD,6)         //Разъем EXTI7//
//#define EXTI8_PIN               pin_id(PORTD,7)         //Разъем EXTI8//
void EXTI9_5_IRQHandler(void)
{


}


//#define EXTI3_PIN               pin_id(PORTC,12)        //Разъем EXTI3//
//#define EXTI10_PIN              pin_id(PORTC,13)        //Разъем EXTI10//*/
//#define EXTI1_PIN               pin_id(PORTA,15)        //Разъем EXTI1///
void EXTI15_10_IRQHandler(void)
{



}

////////////////////////////////////////////////////////////////////////////////
//___________________________________I2C______________________________________//
////////////////////////////////////////////////////////////////////////////////
void delay(__IO uint32_t nCount)
{
  for(; nCount != 0; nCount--);
}
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
//___________________________________ADC______________________________________//
////////////////////////////////////////////////////////////////////////////////
void DMA2_Stream0_IRQHandler(void)
{
DMA2->LIFCR |= DMA_LIFCR_CTCIF0;

}
////////////////////////////////////////////////////////////////////////////////
