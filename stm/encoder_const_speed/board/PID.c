#include "PID.h"


robotstate telega ={.pidenable = 1, .traekenable = 1, .speed = {0,0}, .center = {0,0,0} , .task={0,0,0}, .leftwheel ={0,0,0,0,0} , .rightwheel = {0,0,0,0,0} ,.ready=1};


void pidCalc1(PidStruct *pid_control)  //рассчет ПИД
{
  float error, dif_error;
  error = pid_control->target - pid_control->current;
  dif_error = error - pid_control->prev_error;
  pid_control->prev_error = error;
  pid_control->sum_error += error;

  if (pid_control->sum_error > pid_control->max_sum_error)
    pid_control->sum_error = pid_control->max_sum_error;
  if (pid_control->sum_error < -pid_control->max_sum_error)
    pid_control->sum_error = -pid_control->max_sum_error;

  if (pid_control->pid_on)
  {
    pid_control->output = ((float)(pid_control->p_k * error)+(pid_control->i_k * pid_control->sum_error)+
            (pid_control->d_k * dif_error));


    if (pid_control->output > pid_control->max_output)
      pid_control->output = pid_control->max_output;
    else if (pid_control->output < -pid_control->max_output)
      pid_control->output = -pid_control->max_output;

    if (pid_control->output < pid_control->min_output && pid_control->output > -pid_control->min_output)
      pid_control->output = 0;

    if ((pid_control->output <= pid_control->pid_output_end) &&(
        pid_control->output >= -pid_control->pid_output_end) &&(
        error <= pid_control->pid_error_end) && (error >= -pid_control->pid_error_end))
      pid_control->pid_finish = 1;
    else
      pid_control->pid_finish = 0;
  }
  else
  {
    pid_control->output = 0;
    pid_control->pid_finish = 0;
  }
}

void pidLowLevel1(void) //вычисление ПИД регулятора колес
{
//Low level pid target values are set here__________________________________
  char i;
  for(i =0; i < 2; i++)
  {
    wheelsPidStruct[i].target = regulatorOut[i];//передача требуемых значений от траекторного регулятора
    wheelsPidStruct[1].current = telega.rightwheel.speed; // текущие занчения скоростей колес
    wheelsPidStruct[0].current = telega.leftwheel.speed; // текущие занчения скоростей колес
    if (telega.pidenable ==1 ){
    pidCalc1(&wheelsPidStruct[i]);
    if (i==0) {telega.leftwheel.speedtoircuit = wheelsPidStruct[0].output;}
    if (i==1) {telega.rightwheel.speedtoircuit = -wheelsPidStruct[1].output;}
    }
  }
  //speed[1] = 0;
}

void initRegulators1(void){  // инициализация регуляторов
  int i = 0;
  for (i = 0; i<=2; i++)  // пиды колес
  {
  	wheelsPidStruct[i].p_k = 1.00; //5.0
  	wheelsPidStruct[i].i_k = 2.05; //1
  	wheelsPidStruct[i].d_k = 0.5; //0.5
  	wheelsPidStruct[i].pid_on = 1;
  	wheelsPidStruct[i].pid_error_end  = 3;
  	wheelsPidStruct[i].pid_output_end = 1000;
  	wheelsPidStruct[i].max_sum_error =3000.0;
  	wheelsPidStruct[i].max_output = 400;
  	wheelsPidStruct[i].min_output = 0.01;
  }
}
