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
void TIM2_IRQHandler(void) {
	/* Check if interrupt has occured */
	if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)
	{
		/* Clear interrupt pending bit */
		TIM_ClearITPendingBit(TIM2, TIM_IT_Update);

		/* WHAT EVER YOU NEED TO DO IN THE INTERRUPT HANDLER GOES HERE */
		STM_EVAL_LEDToggle(LED3);
	}
}

int main(void)
{
  /*Analog to digital converter demonstration*/
  RCC_Configuration();

  TIM_Configuration(10000);


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
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6 | RCC_APB2Periph_ADC1 | RCC_APB2Periph_ADC2, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_APB1Periph_DAC  | RCC_AHB1Periph_GPIOC | RCC_AHB1Periph_GPIOA, ENABLE);
	RCC_APB1PeriphClockCmd( RCC_APB1Periph_TIM2 , ENABLE);
}

/**
 * @Brief NVIC Configuration
 */

void NVIC_Configuration(void){
	NVIC_InitTypeDef NVIC_InitStructure;

	/* Configure the Priority Group to 2 bits */
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

	/* Enable the USARTx Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = USART6_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

/*************************************************************************************************
 * @Brief USART6_RECIEVER for MIDI input.														 *
 *																								 *
 * 		USART6 configured as follows:															 *
 *																								 *
 *       - BaudRate = 10500000 baud																 *
 * 		   - Maximum BaudRate that can be achieved when using the Oversampling by 8				 *
 *		     is: (USART APB Clock / 8)															 *
 *			 Example:																			 *
 *			    - (USART6 APB2 Clock / 8) = (84 MHz / 8) = 10500000 baud						 *
 *       - Word Length = 8 Bits								 									 *
 *       - one Stop Bit																			 *
 *       - No parity																			 *
 *       - Hardware flow control disabled (RTS and CTS signals)									 *
 *       - Receive and transmit enabled															 *
 *************************************************************************************************/

void USARTx_Configuration(void) {

	USART_InitTypeDef USART_InitStructure;

	/* Enable the USART OverSampling by 8 */
	USART_OverSampling8Cmd(USART6, ENABLE);

	// USART6 configuration
	USART_InitStructure.USART_BaudRate = 10500000;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_none;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

	USART_Init(USART6, &USART_InitStructure);

	//Enables USART 6 and receiver
	USART_Cmd(USART6, ENABLE);

	//Enable RXNE interrupt
	USART_ITConfig(USART6, USART_IT_RXNE, ENABLE);
}

/* *
 *  @brief Initialize the timer.
 *  @args interval : 84000 = 1ms
 **/

void TIM_Configuration(int interval) {
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;

	/* Configure the timer*/
	TIM_TimeBaseInitStruct.TIM_Period = interval - 1;
	TIM_TimeBaseInitStruct.TIM_Prescaler = 8400 - 1;
	TIM_TimeBaseInitStruct.TIM_ClockDivision = 0;
	TIM_TimeBaseInitStruct.TIM_RepetitionCounter = TIM_CounterMode_Up;

	/* Initialize timer 3*/
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseInitStruct);

	/* Start the count */
	TIM_Cmd(TIM2, ENABLE);

	NVIC_EnableIRQ(TIM2_IRQn); // Enable IRQ for TIM5 in NVIC

	/* Enable timer interrupt */
	TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
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

	//  PA4 and PA5 for DAC.
	GPIO_InitStructure.GPIO_Mode 	= GPIO_Mode_AN;
	GPIO_InitStructure.GPIO_OType 	= GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Pin 	= GPIO_Pin_5 | GPIO_Pin_4;
	GPIO_InitStructure.GPIO_PuPd 	= GPIO_PuPd_NOPULL;

	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/* Configure USART6 Tx and Rx as alternate function push-pull for MIDI Receiver*/
	GPIO_InitStructure.GPIO_Mode 	= GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed 	= GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd 	= GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Pin   	= USARTx_TX_PIN | USARTx_RX_PIN;

	GPIO_Init(GPIOC, &GPIO_InitStructure);

	// Alternate function PC 6-7 to USART_6
	GPIO_PinAFConfig(GPIOC, USARTx_TX_SOURCE, USARTx_TX_AF);
	GPIO_PinAFConfig(GPIOC, USARTx_RX_SOURCE, USARTx_RX_AF);
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
	ADC_InitStructure.ADC_ExternalTrigConvEdge	= ADC_ExternalTrigConvEdge_None;
	ADC_InitStructure.ADC_ExternalTrigConv		= ADC_ExternalTrigConv_T1_CC1;
	ADC_InitStructure.ADC_DataAlign				= ADC_DataAlign_Right;
	ADC_InitStructure.ADC_NbrOfConversion		= 1;

	/* Initialize the two ADC with the same structure */
	ADC_Init(ADC1, &ADC_InitStructure);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_1,1 ,ADC_SampleTime_3Cycles);

	ADC_Init(ADC2, &ADC_InitStructure);
	ADC_RegularChannelConfig(ADC2, ADC_Channel_2,1 ,ADC_SampleTime_3Cycles);

	ADC_Cmd(ADC1, ENABLE);
	ADC_Cmd(ADC2, ENABLE);
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
