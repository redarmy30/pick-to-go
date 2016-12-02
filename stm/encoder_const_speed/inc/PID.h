#ifndef PID_H
#define PID_H

#include "stm32f4xx.h"


float regulatorOut[2] = {0,0};
float motorSpeed[2];




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
  float p_k; //� ����������
  float i_k; //� ����������
  float d_k; //� ����������
  float target; //������� ��������
  float current; //������� (���������� ��������� ����� ����� ������ ��������)
  float prev_error; //���������� �������� ������ (��� � ����������)
  float sum_error; //��������� ������ (��� � ����������)
  float max_sum_error; //������������ ��������� ������ (��� �� � ��������� �� ������ �� ��������� ���� ���������� �������� ���������� ��������)
  float max_output; //������������ �����, ��� �� �������� �� �������� �� �����
  float min_output;
  float cut_output;
  float output; //��������, ��������� ������ ���
  char pid_on; //���/���� ��� ���� ���� �� output ������ ����� 0, ������ ��� ��������� ���������� �������������
  char pid_finish;
  float error_dir;
  float pid_error_end;
  float pid_output_end;
} PidStruct;


PidStruct wheelsPidStruct[2];

void initRegulators1(void);  // ������������� �����������
void pidLowLevel1(void);
void pidCalc1(PidStruct *pid_control);  //������� ���

#endif


