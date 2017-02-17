#include "interapt.h"
#include "TIM.h"

extern robotstate telega;
extern int16_t leftCount;
extern int16_t rightCount;

int counter=1;
int counter1=1;
int s = 255;
int sp = 90;
int hall1=0;
int hall1_0=0;
int hall2=0;
int hall2_0=0;

void USART1_IRQHandler(void){
counter = counter;
USART_ClearITPendingBit(USART1,USART_IT_TC);
}
void USART2_IRQHandler(void){
	// check if the USART1 receive interrupt flag was set
	if (USART_GetITStatus(USART2,USART_IT_TC))
    {
         USART_ClearITPendingBit(USART2,USART_IT_TC);
        if (counter == 0)
        {
            USART_SendData(USART2,(~s)& 0xFF);

        }
        if (counter == 1)
        {
            USART_SendData(USART2,(~s)& 0xFF);

        }
        if (counter == 2)
        {
            USART_SendData(USART2,256);
        }
        if (counter == 3)
        {
            USART_SendData(USART2 , telega.leftwheel.speedtoircuit & 0xFF);
        }
        if (counter == 4)
        {
            USART_SendData(USART2,(telega.leftwheel.speedtoircuit >> 8) & 0xFF);
        }
        if (counter == 5)
        {
            USART_SendData(USART2,telega.leftwheel.speedtoircuit & 0xFF);
        }
        if (counter == 6)
        {
            USART_SendData(USART2,(telega.leftwheel.speedtoircuit >> 8) & 0xFF);
        }
        if (counter == 7)
        {
            USART_SendData(USART2,85);
             s--;
            if (s==0) {s=255;}
            //if (k ==9) k=0;
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
            USART_SendData(USART6,telega.rightwheel.speedtoircuit & 0xFF);
        }
        if (counter1 == 4)
        {
            USART_SendData(USART6,(telega.rightwheel.speedtoircuit >> 8) & 0xFF);
        }
        if (counter1 == 5)
        {
            USART_SendData(USART6,telega.rightwheel.speedtoircuit & 0xFF);
        }
        if (counter1 == 6)
        {
            USART_SendData(USART6,(telega.rightwheel.speedtoircuit >> 8) & 0xFF);
        }
        if (counter1 == 7)
        {
            USART_SendData(USART6,85);

            if (sp==92) {sp=90;}
            else  {sp=92;}
        }
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

  }
  /* Test on DMA Stream Transfer Complete interrupt */
  if(DMA_GetITStatus(DMA2_Stream0, DMA_IT_TCIF0))
  {
    /* Clear DMA Stream Transfer Complete interrupt pending bit */
    DMA_ClearITPendingBit(DMA2_Stream0, DMA_IT_TCIF0);
  }
}

void TIM6_DAC_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM6, TIM_IT_Update) != RESET)
    {
        TIM_ClearITPendingBit(TIM6, TIM_IT_Update);
        encodersRead(); // update encoders counters
        telega.leftwheel.speed = leftCount * WHEELDIAM * PI / TICKSPERROTATION /TIM6_TIME ; //
        telega.rightwheel.speed = -rightCount* WHEELDIAM * PI / TICKSPERROTATION /TIM6_TIME ;
        pidLowLevel1();
        checkposition();
    }
}

void TIM7_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM7, TIM_IT_Update) != RESET)
    {
        TIM_ClearITPendingBit(TIM7, TIM_IT_Update);
    }
}


void TIM2_IRQHandler(void)
{

    if (TIM_GetITStatus(TIM2, TIM_IT_CC1) != RESET)
  {
    TIM_ClearITPendingBit(TIM2, TIM_IT_CC1);
  }
}


void rotateMe (float  *angle) {
    if (telega.traekenable == 1){
        if (fabs(*angle) >0.01){
            telega.leftwheel.task = (*angle)/360 * PI * DISTANCEBTWWHEELS;//      fi*57.3*L/2
            telega.rightwheel.task = -telega.leftwheel.task;
            *angle =0;
        }
    }
}


void GoForward(float *point)
{
    if (telega.traekenable == 1){
        if (fabs(*point)>0.001){
            telega.leftwheel.task  = (*point) ; //   mb wrong //R*2*pi* (ticsperrotation)
            telega.rightwheel.task  = telega.leftwheel.task ;
            *point=0;
        }
    }
}

void checkposition(void)
{
    if (telega.traekenable == 1)
        {
            telega.leftwheel.task +=leftCount * WHEELDIAM * PI / TICKSPERROTATION; //tics
            if (fabs(telega.leftwheel.task )<=0.03) // если осталось ехать меньше 10см остановиться
                 { //if (fabs(regulatorOut[0])>0.51)
                     //regulatorOut[0] -= ((regulatorOut[0]> 0) - (regulatorOut[0]< 0))*0.5;
                     regulatorOut[1] =0;
                 }
            telega.rightwheel.task +=rightCount * WHEELDIAM * PI / TICKSPERROTATION; //tics
            if (fabs(telega.rightwheel.task )<=0.03) // если осталось ехать меньше 10см остановиться
                {   //if ( fabs(regulatorOut[1])>0.51)
                    //regulatorOut[1] -= ((regulatorOut[1]> 0) - (regulatorOut[1]< 0))*0.5;
                    regulatorOut[0]=0;
                }
            if (fabs(telega.leftwheel.task ) >= 0.03)
                {
                    if (fabs(regulatorOut[1])<1)
                    regulatorOut[1]+=((telega.leftwheel.task > 0) - (telega.leftwheel.task < 0)) * 0.1;  // +-1
                }
            if (fabs(telega.rightwheel.task ) >= 0.03)
                {   if (fabs(regulatorOut[0]) < 1)
                    regulatorOut[0]+= ((telega.rightwheel.task > 0) - (telega.rightwheel.task < 0)) * -0.1; // +-1
                }

            if ((fabs(telega.leftwheel.task )<=0.1) &(fabs(telega.rightwheel.task )<=0.1)) {telega.ready = 1;} //movement done
            if ((fabs(telega.leftwheel.task )>0.1) | (fabs(telega.rightwheel.task )>0.1)) {telega.ready = 0;} // moving
        }
}

