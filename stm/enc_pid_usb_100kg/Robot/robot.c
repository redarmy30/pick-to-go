#include "robot.h"
#include "pins.h"
#include "usart.h"
#include "usbd_cdc_core.h"
#include "usbd_usr.h"
#include "usb_conf.h"
#include "usbd_desc.h"
#include "stm32fxxx_it.h"
#include "usbd_cdc_vcp.h"
#include "string.h"
#include "regulator.h"
#include "interrupts.h"
#include "Board.h"
#include "Communication.h"
#include "manipulators.h"
#include "PID.h"
int contrr=0;
//float distanceData[3][4] = {0,0,0,0,0,0,0,0,0,0,0,0};
float distanceData[3][6] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
float distanceFromIR;
bool flag = 1;
uint16_t distance_digital[10] = {0,0,0,0,0,0,0,0,0,0};
uint16_t distance_digital1[10] = {0,0,0,0,0,0,0,0,0,0};
float vTargetGlob_last[3]={0,0,0};
float robotCoordTarget[3] = {0,0,0}; // Целевые координаты робота в глоб сис-ме координат
float robotSpeedTarget[3] = {0,0,0}; // Целевые скорости робота в глоб сис-ме координат
//float motorSpeed[4];                // скорости моторов
float motorCoord[4] = {0,0,0};      // общий пройденный колесом путь
float robotCoord[3] = {0,0,0};       // Координаты робота по показаниям измерительной тележки
float robotSpeed[3] = {0,0,0};       // скорость робота по показаниям измерительной тележки
robStateStruct curState = {1, 1, 1, 0, 0};    // состояние регуляторов активен-1/неактвен -0
encOutPackStruct outEnc;              //буфер данных отправляемых измерительной тележке

uint32_t * encCnt[4] ={ENCODER1_CNT, ENCODER2_CNT, ENCODER3_CNT, ENCODER4_CNT};  //массив указателей на счетчики энкодеров колес
char  WHEELS[4]= {WHEEL1_CH, WHEEL2_CH, WHEEL3_CH, NO_MOTOR}; //каналы подкючения колес

//extern CDC_IF_Prop_TypeDef  APP_FOPS;

char execCommand(InPackStruct* cmd) //обработать входящую команду
{
    switch(cmd->command)
    {
      case 0x01: //Эхо
        {
         char *key=  cmd->param;
         contrr=contrr+1;

          if ((key[0] =='E')&&(key[1] =='C')&&(key[2] =='H')&&(key[3] =='O') )
          {
            char * str ="Pick2GO ROBOT!";
            sendAnswer(cmd->command, str, strlen(str)+1);
            }
          }
      break;

      case 0x02:  //Установить текущие координаты
      {
          float *(temp) ={(float*)cmd->param};

          robotCoord[0]= temp[0];
          robotCoord[1]= temp[1];
          robotCoord[2]= temp[2];

          //points[0].center[0]= temp[0];
          //points[0].center[1]= temp[1];
          //points[0].center[2]= temp[2];

          //CreatePath(&points[0], &points[0], &curPath);
          char * str ="Ok";
          sendAnswer(cmd->command,str, 3);
    }
      break;
        default:
        return 0;
      break;
    }

}



void checkCollisionAvoid_small(float * rV, float* vTargetGlob)
{

          float realRad        = robotCoord[2];

  //float Ml[4][2]     = {(sinus+cosinus), (cosinus-sinus), (cosinus-sinus), -(sinus+cosinus), (cosinus-sinus), -(sinus+cosinus), (sinus+cosinus), (cosinus-sinus)};
 // float Mfi[4]       = {-(0.14), -(0.14),-(0.14), -(0.14)};  //матрица расчета угловой скорости
            float Mrot[3][3]   = {cos(realRad) , sin(realRad), 0,
                        -sin(realRad), cos(realRad), 0,
                                    0,            0, 1};  //матрица пересчета глобальных скоростей в локальные
            float localVelocity[3];
            matrixMultiplyM2M(&Mrot[0][0], 3, 3, vTargetGlob, 3, 1, &localVelocity[0]);//Ml*Velocity speed in local coordinate system

    //if (*flag) {memcpy( vTargetGlob_last, vTargetGlob, sizeof( float)*3);}
//ang3 = adcData[3] * 0.0822 * 2.54;
    //if (fabs( vTargetGlob[2])>0.05 &&((distanceData[1][5]<=threshhold ) || (distanceData[2][5]<=threshhold )||distanceData[0][5]<=threshhold))
     //   {stopmove();}
    //if ((vTargetGlob[0] >0) && (vTargetGlob[1]>0) && (distanceData[1][5]<=threshhold )){
        //stopmove();
      //  stopmove();
       // }

     if ((localVelocity[0]>0) && (localVelocity[1]<0) && ((distanceData[0][5]<=threshhold ) || (distanceData[1][5]<=threshhold ) || (distance_digital[0] == 0))){
            stopmove();}

    else if ((localVelocity[0]<=0) && (localVelocity[1]<=0) && (distanceData[0][5]<=threshhold || (distance_digital[0] == 0))  ) {
        stopmove();}
    else if ((localVelocity[0]<=0) && (localVelocity[1] >0) && ((distanceData[2][5]<=threshhold ) || (distance_digital1[0] == 0))) {
        stopmove();}
    else {
        //curState.trackEn = 1;
        //*flag = 1;
        }
    //*flag = 0;
}


void stopmove(){
        //curState.trackEn=0;
        vTargetGlob[2]=0.0;
        vTargetGlob[0]=0.0;
        vTargetGlob[1]=0.0;
        vTargetGlobF[2]=0.0;
        vTargetGlobF[0]=0.0;
        vTargetGlobF[1]=0.0;
        }

void takeadc(float distanceData[][6],int adc_number1,int adc_number2,int adc_number3)
{
    distanceData[0][0]=distanceData[0][1];
    distanceData[0][1]=distanceData[0][2];
    distanceData[0][2]=distanceData[0][3];
    distanceData[0][3]=distanceData[0][4];

    distanceData[0][4] = adcData[adc_number1]* 0.0822 * 2.54;
    distanceData[0][5] = (distanceData[0][2]  + distanceData[0][1]  + distanceData[0][0] + distanceData[0][3]+distanceData[0][4] ) / 5.0;

    distanceData[1][0]=distanceData[1][1];
    distanceData[1][1]=distanceData[1][2];
    distanceData[1][2]=distanceData[1][3];
    distanceData[1][3]=distanceData[1][4];

    distanceData[1][4] = adcData[adc_number2]* 0.0822 * 2.54;
    distanceData[1][5] = (distanceData[1][2]  + distanceData[1][1]  + distanceData[1][0] + distanceData[1][3]+distanceData[1][4] ) / 5.0;

    distanceData[2][0]=distanceData[2][1];
    distanceData[2][1]=distanceData[2][2];
    distanceData[2][2]=distanceData[2][3];
    distanceData[2][3]=distanceData[2][4];
    distanceData[2][4] = adcData[adc_number3]* 0.0822 * 2.54;
    distanceData[2][5] = (distanceData[2][2]  + distanceData[2][1]  + distanceData[2][0] + distanceData[2][3]+distanceData[2][4] ) / 5.0;




    if (adcData[4]> 0.15*4096.0/3.0)
    {
      distanceFromIR = ((20.0 / ((3.0 * adcData[4] / 4096.0) - 0.15)) );
    } else
    distanceFromIR = (20.0 / ((0.01))) ;




    distance_digital[1]=distance_digital[2];
    distance_digital[2]=distance_digital[3];
    distance_digital[3]=distance_digital[4];
    distance_digital[4]=distance_digital[5];

    distance_digital[5]=distance_digital[6];
    distance_digital[6]=distance_digital[7];
    distance_digital[7]=distance_digital[8];

    distance_digital[8] = pin_val (EXTI5_PIN);
    //distance_digital[0]= distance_digital[1]*distance_digital[2]*distance_digital[3]*distance_digital[4]*distance_digital[5]*distance_digital[6];
    int i =0;
    for (i = 1; i <= 5; i++) // 0-4
    {
        if (distance_digital[i]==0)
        {
            distance_digital[9]=1; //flag
        }
    }
    if (distance_digital[9]==1)
    {distance_digital[0]=0;
     distance_digital[9]=0;}
    else
     {distance_digital[0]=1;
     distance_digital[9]=0;
     }

    distance_digital1[1]=distance_digital1[2];
    distance_digital1[2]=distance_digital1[3];
    distance_digital1[3]=distance_digital1[4];
    distance_digital1[4]=distance_digital1[5];

    distance_digital1[5]=distance_digital1[6];
    distance_digital1[6]=distance_digital1[7];
    distance_digital1[7]=distance_digital1[8];

    distance_digital1[8] = pin_val (EXTI1_PIN);
    //distance_digital[0]= distance_digital[1]*distance_digital[2]*distance_digital[3]*distance_digital[4]*distance_digital[5]*distance_digital[6];

    for (i = 1; i <= 5; i++) // 0-4
    {
        if (distance_digital1[i]==0)
        {
            distance_digital1[9]=1; //flag
        }
    }
    if (distance_digital1[9]==1)
    {distance_digital1[0]=0;
     distance_digital1[9]=0;}
    else
     {distance_digital1[0]=1;
     distance_digital1[9]=0;
     }



}

/*void soft_delay(long int ticks)
{
    for(; ticks > 0; ticks-- );
}
    uint16_t stVal = 0;
    uint16_t finalVal = 0;
    uint16_t curLoad;*/
//______________________________________________________//

