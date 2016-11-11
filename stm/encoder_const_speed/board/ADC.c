#include "ADC.h"

void ADC_Configuration(void)
{
  ADC_CommonInitTypeDef ADC_CommonInitStructure;
  ADC_InitTypeDef ADC_InitStructure;

  /* ADC Common Init */
  ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;
  ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div2;
  ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
  ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
  ADC_CommonInit(&ADC_CommonInitStructure);

  ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
  ADC_InitStructure.ADC_ScanConvMode = ENABLE; // 1 Channel
  ADC_InitStructure.ADC_ContinuousConvMode = ENABLE; // Conversions Triggered
  ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
  ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T2_TRGO;
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
  ADC_InitStructure.ADC_NbrOfConversion = 2;
  ADC_Init(ADC1, &ADC_InitStructure);

  /* ADC1 regular channel 11 configuration */
   //ADC_RegularChannelConfig(ADC1, ADC_Channel_11, 1, ADC_SampleTime_15Cycles); // PC1
   ADC_RegularChannelConfig(ADC1,ADC_Channel_10,1,ADC_SampleTime_480Cycles);
   //ADC_RegularChannelConfig(ADC1,ADC_Channel_12,2,ADC_SampleTime_480Cycles);

  /* Enable DMA request after last transfer (Single-ADC mode) */
  ADC_DMARequestAfterLastTransferCmd(ADC1, ENABLE);


  /* Enable ADC1 DMA */
  ADC_DMACmd(ADC1, ENABLE);

  /* Enable ADC1 */
  ADC_Cmd(ADC1, ENABLE);
  ADC_SoftwareStartConv(ADC1);
}

void ADC_Configuration_new(void){
   ADC_DeInit();
   ADC_InitTypeDef ADC_InitStructure;
   ADC_CommonInitTypeDef ADC_CommonInitStructure;
   ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
   ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
   ADC_InitStructure.ADC_ContinuousConvMode = ENABLE; //continuous conversion
   ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConvEdge_None;
   ADC_InitStructure.ADC_NbrOfConversion = 2;
   ADC_InitStructure.ADC_ScanConvMode = ENABLE; // 1=scan more that one channel in group
   ADC_Init(ADC1,&ADC_InitStructure);

   ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;
   ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div2;
   ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
   ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
   ADC_CommonInit(&ADC_CommonInitStructure);

   ADC_RegularChannelConfig(ADC1,ADC_Channel_12,2,ADC_SampleTime_480Cycles);
   ADC_RegularChannelConfig(ADC1,ADC_Channel_11,1,ADC_SampleTime_480Cycles);

   ADC_DMARequestAfterLastTransferCmd(ADC1, ENABLE);

   ADC_DMACmd(ADC1, ENABLE); //Enable ADC1 DMA

   ADC_Cmd(ADC1, ENABLE);   // Enable ADC1

   ADC_SoftwareStartConv(ADC1); // Start ADC1 conversion

}


