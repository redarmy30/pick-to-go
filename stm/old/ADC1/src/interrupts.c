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

void TIM6_DAC_IRQHandler() // 100Hz  // ������� ��� ����������� �����
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


void TIM8_UP_TIM13_IRQHandler() // ������� ������������ ����������
{
   // set_pin(PWM_DIR[8]);

  TIM13->SR = 0;
 NVIC_DisableIRQ(TIM6_DAC_IRQn);  //���������� ��� �� ����� �������




    ////////////////////////////////////////////////////////////////////////////////
  NVIC_EnableIRQ(TIM6_DAC_IRQn); //��������� ���
    // reset_pin(PWM_DIR[8]);
}



////////////////////////////////////////////////////////////////////////////////
//__________________________________EXTI______________________________________//
////////////////////////////////////////////////////////////////////////////////


//#define EXTI2_PIN               pin_id(PORTD,0)         //������ EXTI2//
void EXTI0_IRQHandler(void)
{

}

//#define EXTI5_PIN               pin_id(PORTD,1)         //������ EXTI5//
void EXTI1_IRQHandler(void)
{

}

//#define EXTI4_PIN               pin_id(PORTD,2)         //������ EXTI4//
void EXTI2_IRQHandler(void)
{

}

//#define EXTI6_PIN               pin_id(PORTD,3)         //������ EXTI6//
void EXTI3_IRQHandler(void)
{


}

//#define EXTI9_PIN               pin_id(PORTE,4)         //������ EXTI9//
void EXTI4_IRQHandler(void)
{


}

//#define EXTI7_PIN               pin_id(PORTD,6)         //������ EXTI7//
//#define EXTI8_PIN               pin_id(PORTD,7)         //������ EXTI8//
void EXTI9_5_IRQHandler(void)
{


}


//#define EXTI3_PIN               pin_id(PORTC,12)        //������ EXTI3//
//#define EXTI10_PIN              pin_id(PORTC,13)        //������ EXTI10//*/
//#define EXTI1_PIN               pin_id(PORTA,15)        //������ EXTI1///
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
