#ifndef PID_H
#define PID_H

#include "stm32f4xx.h"
float regulatorOut[2];
float motorSpeed[2];

typedef struct{
 float speed;
 float dir;
 float distance;
 int speedtoircuit;
 float task;
 float PassedDist
 } wheel;

typedef struct{
 int pidenable;
 int traekenable;
 int speed[2];
 wheel leftwheel;
 wheel rightwheel;
 float center[3];
 float task[3];
 int ready;
} robotstate;

typedef struct {
 int tps;
 float enspeed;
 float distance;
} encoderStruct;

typedef struct{
 int wheelnumber;
 encoderStruct encoder;
} wheelStruct;




typedef struct
{
  float p_k; //П коэфициент
  float i_k; //И коэфициент
  float d_k; //Д коэфициент
  float target; //Целевое значение
  float current; //Текущее (необходимо обновлять извне перед каждым расчетом)
  float prev_error; //Предыдущее значение ошибки (для Д регелятора)
  float sum_error; //Суммарная ошибка (для И регулятора)
  float max_sum_error; //Максимальная суммарная ошибка (что бы И регулятор не уходил до максимума если невозможно добиться требуемого значения)
  float max_output; //Максимальный выход, что бы поправка не выходила за рамки
  float min_output;
  float cut_output;
  float output; //Поправка, результат работы ПИД
  char pid_on; //Вкл/Выкл ПИД если выкл то output всегда равен 0, однако все остальное продолжает расчитываться
  char pid_finish;
  float error_dir;
  float pid_error_end;
  float pid_output_end;
} PidStruct;

PidStruct wheelsPidStruct[2];

void initRegulators1(void);  // инициализация регуляторов
void pidLowLevel1(void);
void pidCalc1(PidStruct *pid_control);  //рассчет ПИД

#endif


