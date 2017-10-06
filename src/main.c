/*
******************************************************************************
File:     main.c
Info:     Generated by Atollic TrueSTUDIO(R) 6.0.0   2017-10-05

The MIT License (MIT)
Copyright (c) 2009-2016 Atollic AB

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
******************************************************************************
*/

/* Includes */
#include "stm32f4xx.h"
#include "stm32f4_discovery.h"
#include "header.h"

/* Private macro */
/* Private variables */
/* Private function prototypes */
/* Private functions */

/**
**===========================================================================
**
**  Abstract: main program
**
**===========================================================================
*/
void TIM5_IRQHandler(void) {
	/* Check if interrupt has occured */
	if (TIM_GetITStatus(TIM5, TIM_IT_Update) != RESET)
	{
		/* Clear interrupt pending bit */
		TIM_ClearITPendingBit(TIM5, TIM_IT_Update);
		 DAC->DHR12R1 = ADC1->DR;
		 DAC->DHR12R2 = ADC2->DR;
		 ADC_SoftwareStartConv(ADC1);

		 /* WHAT EVER YOU NEED TO DO IN THE INTERRUPT HANDLER GOES HERE */
		 STM_EVAL_LEDToggle(LED3);
	}
}

void ADC_IRQHandler(void) {
 DAC->DHR12R1 = ADC1->DR;
 DAC->DHR12R2 = ADC2->DR;
}

int main(void)
{
  /*Analog to digital converter demonstration*/
  RCC_Configuration();

  GPIO_Configuration();

  ADC_Configuration();

  DAC_Configuration();

  TIM_Configuration(840);

  /* Initialize LEDs */
  STM_EVAL_LEDInit(LED3);


  /* Turn on LEDs */
  STM_EVAL_LEDOn(LED3);

  /* Infinite loop */
  while (1)
  {
  }
}

void RCC_Configuration(void){
	/* Initialize all the peripherals here. */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1 | RCC_APB2Periph_ADC2, ENABLE);
	RCC_AHB1PeriphClockCmd( RCC_AHB1Periph_GPIOA, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_DAC  | RCC_APB1Periph_TIM5 , ENABLE);
}
/* *
 *  @brief Initialize the timer.
 *  @args interval : 84000 = 1ms
 **/

void TIM_Configuration(int interval) {
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
	TIM_OCInitTypeDef TIM_OCInitStruct;

	/* Put your timer initialisation here */

	/* Configure the timer*/
	TIM_TimeBaseInitStruct.TIM_Period = interval - 1;
	TIM_TimeBaseInitStruct.TIM_Prescaler = 0;
	TIM_TimeBaseInitStruct.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInitStruct.TIM_RepetitionCounter = TIM_CounterMode_Up;
	TIM_TimeBaseInitStruct.TIM_RepetitionCounter = 0;

	/* Initialize timer 3*/
	TIM_TimeBaseInit(TIM5, &TIM_TimeBaseInitStruct);

	/* Initialise the compare capture structure */
	TIM_OCInitStruct.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStruct.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStruct.TIM_OutputNState = TIM_OutputNState_Disable;
	TIM_OCInitStruct.TIM_Pulse = 1;
	TIM_OCInitStruct.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OCInitStruct.TIM_OCNPolarity = TIM_OCNPolarity_Low;
	TIM_OCInitStruct.TIM_OCIdleState = TIM_OCIdleState_Reset;
	TIM_OCInitStruct.TIM_OCNIdleState = TIM_OCNIdleState_Set;
	TIM_OC1Init(TIM5, &TIM_OCInitStruct);

	/* Start the count */
	TIM_Cmd(TIM5, ENABLE);
}
/**
 * Initialize DAC for both chnnels
 */
void DAC_Configuration(void){
	DAC_InitTypeDef DAC_InitStructure;

	DAC_InitStructure.DAC_Trigger = DAC_Trigger_None;
	DAC_InitStructure.DAC_WaveGeneration = DAC_WaveGeneration_None;
	DAC_InitStructure.DAC_OutputBuffer = DAC_OutputBuffer_Enable;
	DAC_Init(DAC_Channel_1, &DAC_InitStructure);
	DAC_Init(DAC_Channel_2, &DAC_InitStructure);

	DAC_Cmd(DAC_Channel_1, ENABLE);
	DAC_Cmd(DAC_Channel_2, ENABLE);
}

/**
 * Initialize GPIO for pins to be used
 */
void GPIO_Configuration(void){
	GPIO_InitTypeDef GPIO_InitStructure;

	// We will be sampling Pin 1 and 2 of GPIOA
	GPIO_InitStructure.GPIO_Mode 	= GPIO_Mode_AN;
	GPIO_InitStructure.GPIO_OType 	= GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Pin 	= GPIO_Pin_5 | GPIO_Pin_4 | GPIO_Pin_2 | GPIO_Pin_1;
	GPIO_InitStructure.GPIO_PuPd 	= GPIO_PuPd_NOPULL;

	GPIO_Init(GPIOA, &GPIO_InitStructure);
}

/**
 *  Configure hardware that is common to all three ADC
 */
void ADC_Configuration(void){
	ADC_InitTypeDef 		ADC_InitStructure;
	ADC_CommonInitTypeDef 	ADC_InitCommonStructure;

	/* Fill the data structure common to all  ADCs*/
	ADC_InitCommonStructure.ADC_Mode 	  		= ADC_DualMode_RegSimult;
	ADC_InitCommonStructure.ADC_Prescaler 		= ADC_Prescaler_Div2;
	ADC_InitCommonStructure.ADC_DMAAccessMode 	= ADC_DMAAccessMode_Disabled;

	ADC_CommonInit(&ADC_InitCommonStructure);	// <---------- Initialize the hardware


	/* Fill the structure to initialize the two ADCs */
	ADC_InitStructure.ADC_Resolution 			= ADC_Resolution_12b;
	ADC_InitStructure.ADC_ScanConvMode 			= DISABLE;
	ADC_InitStructure.ADC_ContinuousConvMode	= DISABLE;
	ADC_InitStructure.ADC_ExternalTrigConvEdge	= ADC_ExternalTrigConvEdge_Rising;
	ADC_InitStructure.ADC_ExternalTrigConv		= ADC_ExternalTrigConv_T5_CC1;
	ADC_InitStructure.ADC_DataAlign				= ADC_DataAlign_Right;
	ADC_InitStructure.ADC_NbrOfConversion		= 1;

	/* Initialize the two ADC with the same structure */
	ADC_Init(ADC1, &ADC_InitStructure);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_1,1 ,ADC_SampleTime_3Cycles);

	ADC_Init(ADC2, &ADC_InitStructure);
	ADC_RegularChannelConfig(ADC2, ADC_Channel_2,1 ,ADC_SampleTime_3Cycles);

	ADC_Cmd(ADC1, ENABLE);
	ADC_Cmd(ADC2, ENABLE);

	NVIC_EnableIRQ(ADC_IRQn); 					// Enable IRQ for ADC in NVIC
	ADC_ITConfig(ADC1, ADC_IT_EOC, ENABLE);		// Enable ADC IRQ generation
}

/*
 * Callback used by stm32f4_discovery_audio_codec.c.
 * Refer to stm32f4_discovery_audio_codec.h for more info.
 */
void EVAL_AUDIO_TransferComplete_CallBack(uint32_t pBuffer, uint32_t Size){
  /* TODO, implement your code here */
  return;
}

/*
 * Callback used by stm324xg_eval_audio_codec.c.
 * Refer to stm324xg_eval_audio_codec.h for more info.
 */
uint16_t EVAL_AUDIO_GetSampleCallBack(void){
  /* TODO, implement your code here */
  return -1;
}
