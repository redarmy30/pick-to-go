
#include <stm32f4xx.h>
//#include<stm32f4xx_gpio.h>
#include <misc.h>

void delay(uint32_t i) {
 volatile uint32_t j;
 for (j=0; j!= i * 1000; j++)
  ;
}

int main(void)
{
  GPIO_InitTypeDef PORT;
  //Включаем порты А и С
  RCC_APB2PeriphClockCmd((RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOA) , ENABLE);
  //Настраиваем ноги PC8 и PC9 на выход. Там у нас висят светодиоды
  /*PORT.GPIO_Pin = (GPIO_Pin_9 | GPIO_Pin_8);
  PORT.GPIO_Mode = GPIO_Mode_Out_PP;
  PORT.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_Init( GPIOC , &PORT);*/
  //Порт A настраивать смысла нет, все его ноги по умолчанию входы что нам и нужно
  RCC_APB2PeriphClockCmd(RCC_APB2ENR_ADC1EN, ENABLE); //Включаем тактирование АЦП
  ADC1->CR2 |= ADC_CR2_CAL; //Запуск калибровки АЦП
  while (!(ADC1->CR2 & ADC_CR2_CAL))
    ; //Ожидаем окончания калибровки
  ADC1->SMPR2 |= (ADC_SMPR2_SMP1_2 | ADC_SMPR2_SMP1_1 | ADC_SMPR2_SMP1_0); //Задаем длительность выборки
  ADC1->CR2 |= ADC_CR2_JEXTSEL; //Преобразование инжектированной группы
                                                       //запустится установкой бита JSWSTART
  ADC1->CR2 |= ADC_CR2_JEXTTRIG; //Разрешаем внешний запуск инжектированной группы
  ADC1->CR2 |= ADC_CR2_CONT; //Преобразования запускаются одно за другим
  ADC1->CR1 |= ADC_CR1_JAUTO; //Разрешить преобразование инжектированной группы
                                     //после регулярной. Не понятно зачем, но без этого не работает
  ADC1->JSQR |= (1<<15); //Задаем номер канала (выбран ADC1)
  ADC1->CR2 |= ADC_CR2_ADON;//Теперь включаем АЦП
  ADC1->CR2 |= ADC_CR2_JSWSTART; //Запуск преобразований
  while (!(ADC1->SR & ADC_SR_JEOC)) //ждем пока первое преобразование завершится
    ;
  //Теперь можно читать результат из JDR1
  uint32_t adc_res; //Использовал переменную для отладки. Можно и без неё
  while(1)
  {
    adc_res=ADC1->JDR1;
    delay(adc_res); //Исходя из значения АЦП делаем задержку
    GPIO_WriteBit(GPIOC,GPIO_Pin_8,Bit_RESET); //Гасим диод...
    GPIO_WriteBit(GPIOC,GPIO_Pin_9,Bit_SET); // Зажигаем диод...
    adc_res=ADC1->JDR1;
    delay(adc_res); //Всё повторяется...
    GPIO_WriteBit(GPIOC,GPIO_Pin_9,Bit_RESET);
    GPIO_WriteBit(GPIOC,GPIO_Pin_8,Bit_SET);
  }
}
