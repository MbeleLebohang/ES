/**
*****************************************************************************
**
**  File        : ECSA ELO PRAC
**
**  Abstract    : main function.
**
**  Functions   : main
**
**  Environment : Atollic TrueSTUDIO(R)
**                STMicroelectronics STM32F4xx Standard Peripherals Library
**
**  Distribution: The file is distributed “as is,” without any warranty
**                of any kind.
**
**  (c)Copyright Atollic AB.
**  You may use this file as-is or modify it according to the needs of your
**  project. Distribution of this file (unmodified or modified) is not
**  permitted. Atollic AB permit registered Atollic TrueSTUDIO(R) users the
**  rights to distribute the assembled, compiled & linked contents of this
**  file as part of an application binary file, provided that it is built
**  using the Atollic TrueSTUDIO(R) toolchain.
**
**
*****************************************************************************
*/

/* Includes */
#include "stm32f4xx.h"
#include "stm32f4_discovery.h"
#include <math.h>
#include "header.h"

/* Private macro */
#define DACBUFFERSIZE 		256
#define WAVEFREQ			1000 /* 1KHz */
#define TIMER6_PRESCALER	2 	/* produces a 42MHz tick */
#define TIMER_CLOCK			84E6 /* TIM6 runs at 84MHz */
/* Private variables */
uint16_t DACBuffer1[DACBUFFERSIZE]; 	/* Array for  waveform 1*/
char MIDI_BYTEx;
char MIDI_NOTE_ON;
char Midi_Bytes[3];

/* Private function prototypes */


/*******************************************************************************
* Function Name  : TIM2_IRQHandler
* Description    : This function handles TIM2 global interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void TIM2_IRQHandler(void)
{
  if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)
  {
    TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
    //Do stuff here
  }
}


/**
**===========================================================================
**
**  Abstract: main program
**
**===========================================================================
*/
int main(void)
{
	uint32_t fTimer;
	uint32_t timerFreq;
	uint16_t timerPeriod;
	uint16_t n;

	/* Create wave table for sinewave */
	for (n = 0; n<DACBUFFERSIZE; n++)
	{
		DACBuffer1[n] = (uint16_t)((4095+1)/2.0)*( sin( M_TWOPI*n/DACBUFFERSIZE) + 1.0 );
	}

	/* Calculate frequency of timer */
	fTimer = WAVEFREQ * DACBUFFERSIZE;

	/* Calculate Tick Rate */
	timerFreq = TIMER_CLOCK / TIMER6_PRESCALER; /* Timer tick is in Hz */

	/* Calculate period of Timer */
	timerPeriod = (uint16_t)( timerFreq / fTimer );

	/* System Clocks Configuration */
	RCC_Configuration();

	/* NVIC configuration */
	NVIC_Configuration();

	/* Configure the GPIO ports */
	GPIO_Configuration();

	/* Configure the USART6 MIDI receiver */
	USARTx_Configuration();

	/* Timer Configuration */
	TIM_Configuration(timerPeriod);

	/* DAC Configuration */
	DAC_Configuration();

	/* DMA Config --> Takes in a pointer to the waveform buffer */
	DMA_Configuration ( DACBuffer1 );

	/* Infinite loop */
	while (1)
	{
	  /* Do stuff here... */

	}
}

/**
  * @brief  Configures the different system clocks.
  * @param  None
  * @retval : None
  */
void RCC_Configuration(void)
{
	/* Initialize all the peripherals here. */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6 | RCC_APB2Periph_ADC1 | RCC_APB2Periph_ADC2, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1 | RCC_APB1Periph_DAC  | RCC_AHB1Periph_GPIOC | RCC_AHB1Periph_GPIOA, ENABLE);
	RCC_APB1PeriphClockCmd( RCC_APB1Periph_DAC | RCC_APB1Periph_TIM6 , ENABLE);
}

/**
  * @brief  Configures the nested vectored interrupt controller.
  * @param  None
  * @retval : None
  */
void NVIC_Configuration(void)
{
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
	USART_InitStructure.USART_BaudRate = 31250;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

	USART_Init(USART6, &USART_InitStructure);

	//Enables USART 6 and receiver
	USART_Cmd(USART6, ENABLE);

	//Enable RXNE interrupt
	USART_ITConfig(USART6, USART_IT_RXNE, ENABLE);
}
void USART6_IRQHandler() {
	if (USART_GetITStatus(USART6, USART_IT_RXNE) != RESET)
	{
		USART_ClearITPendingBit(USART6, USART_IT_RXNE);
		//Do stuff here
		if (USART6->DR/16 == 0x9) {
				//If it has just received a new status byte
				MIDI_BYTEx = 1;
				MIDI_NOTE_ON = 1;
			}
			else {
				//If not receiving a status byte and byte_no > 3, assume running_status byte(s)
				//If some other status byte
				if ((USART6->DR >> 7) == 1){
					MIDI_NOTE_ON = 0;
				}

				if (MIDI_BYTEx > 3){
					MIDI_BYTEx = 2;			//Running status
				}
			}

			//Read in byte
			Midi_Bytes[MIDI_BYTEx-1] = USART6->DR;

			if (MIDI_BYTEx == 3 && MIDI_NOTE_ON == 1 && Midi_Bytes[2] != 0) {
				//If the current command is NOTE ON
				if (Midi_Bytes[0]/16 == 0x9) {
					//Finish reading block
					// calculate the new ARR
					char data1 =  Midi_Bytes[0];
					data1 =  Midi_Bytes[1];
					data1 =  Midi_Bytes[2];
					data1 =  Midi_Bytes[0];

					STM_EVAL_LEDToggle(LED3);

				}
			}
			MIDI_BYTEx++;

	}
}
/**
  * @brief  Configures the different GPIO ports.
  * @param  None
  * @retval : None
  */
void GPIO_Configuration(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	/* Pack the struct */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;

	/* Call Init function */
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
  * @brief  Configures the DMA.
  * @param  None
  * @retval : None
  */
void DMA_Configuration( uint16_t* wavBuffer )
{
	DMA_InitTypeDef DMA_InitStructure;

	//Initialize the structure to default values
	DMA_StructInit(&DMA_InitStructure);

	DMA_InitStructure.DMA_Channel = DMA_Channel_7;
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(DAC_BASE + 0x08);  //DAC channel1 12-bit right-aligned data holding register (ref manual pg. 264)
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)wavBuffer;
	DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
	DMA_InitStructure.DMA_BufferSize = DACBUFFERSIZE;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;

	/* Call Init function */
	DMA_Init(DMA1_Stream5, &DMA_InitStructure);

	/* Enable DMA */
	DMA_Cmd(DMA1_Stream5, ENABLE);

}

/**
  * @brief  Configures the Timers.
  * @param  wavePeriod (period of timer), preScaler (prescaler for timer)
  * @retval : None
  */
void TIM_Configuration(uint16_t wavPeriod)
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStruct;

	/* pack Timer struct */
	TIM_TimeBaseStruct.TIM_Period = wavPeriod-1;
	TIM_TimeBaseStruct.TIM_Prescaler = TIMER6_PRESCALER -1;
	TIM_TimeBaseStruct.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStruct.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStruct.TIM_RepetitionCounter = 0x0000;

	/* Call init function */
	TIM_TimeBaseInit(TIM6, &TIM_TimeBaseStruct);

	/* Select Timer to trigger DAC */
	TIM_SelectOutputTrigger(TIM6, TIM_TRGOSource_Update);

	/* TIM6 enable counter */
	TIM_Cmd(TIM6, ENABLE);

}

/**
  * @brief  Configures the DAC
  * @param  None
  * @retval : None
  */
void DAC_Configuration(void)
{
	DAC_InitTypeDef DAC_InitStruct;

	/* Initialize the DAC_Trigger member */
	DAC_InitStruct.DAC_Trigger = DAC_Trigger_T6_TRGO;
	/* Initialize the DAC_WaveGeneration member */
	DAC_InitStruct.DAC_WaveGeneration = DAC_WaveGeneration_None;
	/* Initialize the DAC_LFSRUnmask_TriangleAmplitude member */
	DAC_InitStruct.DAC_LFSRUnmask_TriangleAmplitude = DAC_LFSRUnmask_Bit0;
	/* Initialize the DAC_OutputBuffer member */
	DAC_InitStruct.DAC_OutputBuffer = DAC_OutputBuffer_Enable;

	/* Init DAC */
	DAC_Init(DAC_Channel_1, &DAC_InitStruct);

	/* Enable DMA request */
	DAC_DMACmd(DAC_Channel_1, ENABLE);

	/* Enable DAC Channel1: Once the DAC channel1 is enabled, PA.04 is automatically connected to the DAC converter. */
	DAC_Cmd(DAC_Channel_1, ENABLE);

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


/**
  * @brief  Simple delay in ms --> Maybe use for debouncing?
  * @param  integer amount of milliseconds
  * @retval None
  */
void delay_ms(uint32_t milli)
{
	uint32_t delay = milli * 17612;              // approximate loops per ms at 168 MHz, Debug config
	for(; delay != 0; delay--);
}
