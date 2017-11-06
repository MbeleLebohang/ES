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
#include "stm32f4_discovery_audio_codec.h"
#include "stm32f4_discovery_lis302dl.h"
#include <stdio.h>
#include "stm32f4xx_it.h"
#include "header.h"

/* Private macro */
/* Private variables */
char MIDI_BYTEx;
char MIDI_NOTE_ON;
char Midi_Bytes[3];
int CircularBuffer1[1024], bufferPtr = 0;

/* Private function prototypes */
/* Private function prototypes -----------------------------------------------*/
static void MEMS_Configuration(void);
static void EXTILine_Configuration(void);

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
	/*MIDI Receiver*/
	SystemInit();

	RCC_Configuration();

	NVIC_Configuration();

	/* MEMS Accelerometre configure to manage PAUSE, RESUME and Controle Volume operation */
	MEMS_Configuration();

	/* EXTI configue to detect interrupts on Z axis click and on Y axis high event */
	EXTILine_Configuration();

	GPIO_Configuration();
	CODEC_Configuration();
	CS43L22_Configuration();

	/* Fill up the circular buffer*/
	int i;
	for(i = 0; i < 1024; i++){
		if(i < 512){
			CircularBuffer1[i] = 0;
		}
		else{
			CircularBuffer1[i] = 4095;
		}
	}
	/*Initialize the user button*/
	STM_EVAL_PBInit(BUTTON_USER, BUTTON_MODE_GPIO);
	int volume = 50;

	/* Infinite loop */
    while(1)
    {

    	if (SPI_I2S_GetFlagStatus(CODEC_I2S, SPI_I2S_FLAG_TXE))
    	{
    		bufferPtr++;
    		bufferPtr &= 1023;		// Wrap around
    		SPI_I2S_SendData(CODEC_I2S, CircularBuffer1[bufferPtr]);

    	}

    	/* Volume test */
    	if(STM_EVAL_PBGetState(BUTTON_USER)){
    		while(STM_EVAL_PBGetState(BUTTON_USER)){}
    		if(volume > 100){
    			volume = 0;
    		}
    		CODEC_Volume_CTRL(volume++);
    	}

	}
}

/*************************************************************************************************
 * @Brief Clock for different peripherals used for this project.								 *
 *																								 *
 * 		USART6 used for:															 			 *
 *			- Serial communication with MIDI Controller								 			 *																			 *
 * 		GPIOA used for:																			 *
 *		    - DAC output signal																	 *
 *			- I2S_WS signal																		 *
 *		GPIOB used for:																			 *
 *		    - I2C_SDA, I2C_SCL 																	 *																	 *
 *		GPIOC used for:																			 *
 *		    - I2S_MCK, I2S_SCK, I2S_SD 															 *
 *		    - USART Rx and Tx pins														 		 *
 *		GPIOD used for:																			 *
 *		    - Reset pin on CS43L22																 *
 *																								 *
 *		CODEC uses the following peripherals I2C1, I2S and SPI3										 *
 *																								 *
 *************************************************************************************************/
void RCC_Configuration(void){
	/* Initialize all the peripherals here. */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6 | RCC_APB2Periph_ADC1 | RCC_APB2Periph_ADC2, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOB, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC  | RCC_AHB1Periph_GPIOD, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_DAC | RCC_APB1Periph_TIM2 | RCC_APB1Periph_I2C1 | RCC_APB1Periph_SPI3 , ENABLE);

	/* Use PLL module enable I2S peripherals clock for accurate standard audio sampling */
	RCC_PLLI2SCmd(ENABLE);
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

/* *
 *  @brief Initialize peripherals used by the CODEC.
 *  @args none
 **/
void CODEC_Configuration(void){
	I2S_InitTypeDef I2S_InitStructure;
	I2C_InitTypeDef I2C_InitStructure;

	// configure I2S port
	SPI_I2S_DeInit(CODEC_I2S);
	I2S_InitStructure.I2S_AudioFreq = I2S_AudioFreq_48k;
	I2S_InitStructure.I2S_MCLKOutput = I2S_MCLKOutput_Enable;
	I2S_InitStructure.I2S_DataFormat = I2S_DataFormat_16b;
	I2S_InitStructure.I2S_Mode = I2S_Mode_MasterTx;
	I2S_InitStructure.I2S_Standard = I2S_Standard_Phillips;
	I2S_InitStructure.I2S_CPOL = I2S_CPOL_Low;

	I2S_Init(CODEC_I2S, &I2S_InitStructure);

	// configure I2C port
	I2C_DeInit(CODEC_I2C);
	I2C_InitStructure.I2C_ClockSpeed = 100000;
	I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
	I2C_InitStructure.I2C_OwnAddress1 = CORE_I2C_ADDRESS;
	I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
	I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
	I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;

	I2C_Cmd(CODEC_I2C, ENABLE);
	I2C_Init(CODEC_I2C, &I2C_InitStructure);
}

/**
  * @brief  Configure the volune
  * @param  vol: volume value (0-100)
  * @retval None
  */
uint8_t CODEC_Volume_CTRL(uint8_t vol){
  EVAL_AUDIO_VolumeCtl(vol);
  return 0;
}

/**
* @brief  configure the mems accelometer to  Control Volume operation
* @param  None
* @retval None
*/
static void MEMS_Configuration(void)
{
  uint8_t ctrl = 0;

  LIS302DL_InitTypeDef  LIS302DL_InitStruct;
  LIS302DL_InterruptConfigTypeDef LIS302DL_InterruptStruct;

  /* Set configuration of LIS302DL*/
  LIS302DL_InitStruct.Power_Mode = LIS302DL_LOWPOWERMODE_ACTIVE;
  LIS302DL_InitStruct.Output_DataRate = LIS302DL_DATARATE_100;
  LIS302DL_InitStruct.Axes_Enable = LIS302DL_X_ENABLE | LIS302DL_Y_ENABLE | LIS302DL_Z_ENABLE;
  LIS302DL_InitStruct.Full_Scale = LIS302DL_FULLSCALE_2_3;
  LIS302DL_InitStruct.Self_Test = LIS302DL_SELFTEST_NORMAL;
  LIS302DL_Init(&LIS302DL_InitStruct);

  /* Set configuration of Internal High Pass Filter of LIS302DL*/
  LIS302DL_InterruptStruct.Latch_Request = LIS302DL_INTERRUPTREQUEST_LATCHED;
  LIS302DL_InterruptStruct.SingleClick_Axes = LIS302DL_CLICKINTERRUPT_Z_ENABLE;
  LIS302DL_InterruptStruct.DoubleClick_Axes = LIS302DL_DOUBLECLICKINTERRUPT_Z_ENABLE;
  LIS302DL_InterruptConfig(&LIS302DL_InterruptStruct);

  /* Configure Interrupt control register: enable Click interrupt on INT1 and
     INT2 on Z axis high event */
  ctrl = 0x3F;
  LIS302DL_Write(&ctrl, LIS302DL_CTRL_REG3_ADDR, 1);

  /* Enable Interrupt generation on click on Z axis */
  ctrl = 0x50;
  LIS302DL_Write(&ctrl, LIS302DL_CLICK_CFG_REG_ADDR, 1);

  /* Configure Click Threshold on X/Y axis (10 x 0.5g) */
  ctrl = 0xAA;
  LIS302DL_Write(&ctrl, LIS302DL_CLICK_THSY_X_REG_ADDR, 1);

  /* Configure Click Threshold on Z axis (10 x 0.5g) */
  ctrl = 0x0A;
  LIS302DL_Write(&ctrl, LIS302DL_CLICK_THSZ_REG_ADDR, 1);

  /* Enable interrupt on Y axis high event */
  ctrl = 0x4C;
  LIS302DL_Write(&ctrl, LIS302DL_FF_WU_CFG1_REG_ADDR, 1);

  /* Configure Time Limit */
  ctrl = 0x03;
  LIS302DL_Write(&ctrl, LIS302DL_CLICK_TIMELIMIT_REG_ADDR, 1);

  /* Configure Latency */
  ctrl = 0x7F;
  LIS302DL_Write(&ctrl, LIS302DL_CLICK_LATENCY_REG_ADDR, 1);

  /* Configure Click Window */
  ctrl = 0x7F;
  LIS302DL_Write(&ctrl, LIS302DL_CLICK_WINDOW_REG_ADDR, 1);

}

/**
  * @brief  Configures EXTI Line0 (connected to PA0 pin) in interrupt mode
  * @param  None
  * @retval None
  */
static void EXTILine_Configuration(void)
{
  GPIO_InitTypeDef   GPIO_InitStructure;
  NVIC_InitTypeDef   NVIC_InitStructure;
  EXTI_InitTypeDef   EXTI_InitStructure;
  /* Enable GPIOA clock */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
  /* Enable SYSCFG clock */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
  /* Configure PE0 and PE1 pins as input floating */
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1;
  GPIO_Init(GPIOE, &GPIO_InitStructure);

  /* Connect EXTI Line to PE1 pins */
  SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOE, EXTI_PinSource1);

  /* Configure EXTI Line1 */
  EXTI_InitStructure.EXTI_Line = EXTI_Line1;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);

  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_3);

  /* Enable and set EXTI Line0 Interrupt to the lowest priority */
  NVIC_InitStructure.NVIC_IRQChannel = EXTI1_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
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

	/* Configure USART6 Tx and Rx as alternate function push-pull for MIDI Receiver */
	GPIO_InitStructure.GPIO_Mode 	= GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed 	= GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd 	= GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Pin   	= USARTx_TX_PIN | USARTx_RX_PIN;

	GPIO_Init(GPIOC, &GPIO_InitStructure);

	/* Alternate function PC 6-7 to USART_6 */
	GPIO_PinAFConfig(GPIOC, USARTx_TX_SOURCE, USARTx_TX_AF);
	GPIO_PinAFConfig(GPIOC, USARTx_RX_SOURCE, USARTx_RX_AF);

	/* Configure CODEC_RESET_PIN */
	GPIO_InitStructure.GPIO_Pin = CODEC_RESET_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

	GPIO_Init(GPIOD, &GPIO_InitStructure);

	/* Configure I2C Pins */
	GPIO_InitStructure.GPIO_Pin = I2C_SDA_PIN | I2C_SCL_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;

	GPIO_Init(GPIOB, &GPIO_InitStructure);

	/* Alternate function PC 6-7 to I2C */
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_I2C1);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource9, GPIO_AF_I2C1);

	/* Configure I2S pins */
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Pin = I2S3_SCLK_PIN | I2S3_SD_PIN | I2S3_MCLK_PIN;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = I2S3_WS_PIN;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/* Configure output ports to AF */
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource4, GPIO_AF_SPI3);
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource7, GPIO_AF_SPI3);
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource10, GPIO_AF_SPI3);
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource12, GPIO_AF_SPI3);

	/*  Reset the CODEC so that I2S and I2c can be configured. */
	GPIO_ResetBits(GPIOD, CODEC_RESET_PIN);

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

void USART6_IRQHandler() {
	// Read the data
	uint16_t data = USART_ReceiveData(USART6);

	if (data/16 == 0x9) {
		//If it has just received a new status byte
		MIDI_BYTEx = 1;
		MIDI_NOTE_ON = 1;
	}
	else {
		//If not receiving a status byte and byte_no > 3, assume running_status byte(s)
		//If some other status byte
		if ((data >> 7) == 1){
			MIDI_NOTE_ON = 0;
		}

		if (MIDI_BYTEx > 3){
			MIDI_BYTEx = 2;			//Running status
		}
	}

	//Read in byte
	Midi_Bytes[MIDI_BYTEx-1] = USART_ReceiveData(USART6);
	if (MIDI_BYTEx == 3 && MIDI_NOTE_ON == 1 && Midi_Bytes[2] != 0) {
		//If the current command is NOTE ON
		if (Midi_Bytes[0]/16 == 0x9) {
			//Finish reading block

		}
	}
	MIDI_BYTEx++;
}
void CS43L22_Configuration(void)
{
	uint32_t delaycount;
	uint8_t cmdBuffer[5];

	uint8_t regValue = 0xFF;

	GPIO_SetBits(GPIOD, CODEC_RESET_PIN);
	delaycount = 1000000;
	while (delaycount > 0)
	{
		delaycount--;
	}
	//keep codec OFF
	cmdBuffer[0] = CODEC_MAP_PLAYBACK_CTRL1;
	cmdBuffer[1] = 0x01;
	Send_CODEC_Command(cmdBuffer, 2);

	//begin initialization sequence (p. 32)
	cmdBuffer[0] = 0x00;
	cmdBuffer[1] = 0x99;
	Send_CODEC_Command(cmdBuffer, 2);

	cmdBuffer[0] = 0x47;
	cmdBuffer[1] = 0x80;
	Send_CODEC_Command(cmdBuffer, 2);

	regValue = Read_CODEC_Register(0x32);

	cmdBuffer[0] = 0x32;
	cmdBuffer[1] = regValue | 0x80;
	Send_CODEC_Command(cmdBuffer, 2);

	regValue = Read_CODEC_Register(0x32);

	cmdBuffer[0] = 0x32;
	cmdBuffer[1] = regValue & (~0x80);
	Send_CODEC_Command(cmdBuffer, 2);

	cmdBuffer[0] = 0x00;
	cmdBuffer[1] = 0x00;
	Send_CODEC_Command(cmdBuffer, 2);
	//end of initialization sequence

	cmdBuffer[0] = CODEC_MAP_PWR_CTRL2;
	cmdBuffer[1] = 0xAF;
	Send_CODEC_Command(cmdBuffer, 2);

	cmdBuffer[0] = CODEC_MAP_PLAYBACK_CTRL1;
	cmdBuffer[1] = 0x70;
	Send_CODEC_Command(cmdBuffer, 2);

	cmdBuffer[0] = CODEC_MAP_CLK_CTRL;
	cmdBuffer[1] = 0x81; //auto detect clock
	Send_CODEC_Command(cmdBuffer, 2);

	cmdBuffer[0] = CODEC_MAP_IF_CTRL1;
	cmdBuffer[1] = 0x07;
	Send_CODEC_Command(cmdBuffer, 2);

	cmdBuffer[0] = 0x0A;
	cmdBuffer[1] = 0x00;
	Send_CODEC_Command(cmdBuffer, 2);

	cmdBuffer[0] = 0x27;
	cmdBuffer[1] = 0x00;
	Send_CODEC_Command(cmdBuffer, 2);

	cmdBuffer[0] = 0x1A | CODEC_MAPBYTE_INC;
	cmdBuffer[1] = 0x0A;
	cmdBuffer[2] = 0x0A;
	Send_CODEC_Command(cmdBuffer, 3);

	cmdBuffer[0] = 0x1F;
	cmdBuffer[1] = 0x0F;
	Send_CODEC_Command(cmdBuffer, 2);

	cmdBuffer[0] = CODEC_MAP_PWR_CTRL1;
	cmdBuffer[1] = 0x9E;
	Send_CODEC_Command(cmdBuffer, 2);

	/* Enable SPI  */
	I2S_Cmd(CODEC_I2S, ENABLE);
}

/**
 * @Brief Send commands to CS43L22
 */
void Send_CODEC_Command(uint8_t controlBytes[], uint8_t numBytes)
{
	uint8_t bytesSent=0;

	//just wait until no longer busy
	while (I2C_GetFlagStatus(CODEC_I2C, I2C_FLAG_BUSY)){}

	I2C_GenerateSTART(CODEC_I2C, ENABLE);
	//wait for generation of start condition
	while (!I2C_GetFlagStatus(CODEC_I2C, I2C_FLAG_SB)){}

	I2C_Send7bitAddress(CODEC_I2C, CODEC_I2C_ADDRESS, I2C_Direction_Transmitter);
	//wait for end of address transmission
	while (!I2C_CheckEvent(CODEC_I2C, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)){}

	while (bytesSent < numBytes)
	{
		I2C_SendData(CODEC_I2C, controlBytes[bytesSent]);
		bytesSent++;
		//wait for transmission of byte
		while (!I2C_CheckEvent(CODEC_I2C, I2C_EVENT_MASTER_BYTE_TRANSMITTING)){}
	}
	//wait until it's finished sending before creating STOP
	while(!I2C_GetFlagStatus(CODEC_I2C, I2C_FLAG_BTF)){}
	I2C_GenerateSTOP(CODEC_I2C, ENABLE);

}

/**
 * @Brief Read CS43L22 Register
 */
uint8_t Read_CODEC_Register(uint8_t mapbyte)
{
	uint8_t receivedByte = 0;

	//just wait until no longer busy
	while (I2C_GetFlagStatus(CODEC_I2C, I2C_FLAG_BUSY)){}

	I2C_GenerateSTART(CODEC_I2C, ENABLE);
	//wait for generation of start condition
	while (!I2C_GetFlagStatus(CODEC_I2C, I2C_FLAG_SB)){}

	I2C_Send7bitAddress(CODEC_I2C, CODEC_I2C_ADDRESS, I2C_Direction_Transmitter);
	//wait for end of address transmission
	while (!I2C_CheckEvent(CODEC_I2C, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)){}

	I2C_SendData(CODEC_I2C, mapbyte); //sets the transmitter address
	//wait for transmission of byte
	while (!I2C_CheckEvent(CODEC_I2C, I2C_EVENT_MASTER_BYTE_TRANSMITTING)){}
	I2C_GenerateSTOP(CODEC_I2C, ENABLE);

	//just wait until no longer busy
	while (I2C_GetFlagStatus(CODEC_I2C, I2C_FLAG_BUSY)){}

	I2C_AcknowledgeConfig(CODEC_I2C, DISABLE);

	I2C_GenerateSTART(CODEC_I2C, ENABLE);
	//wait for generation of start condition
	while (!I2C_GetFlagStatus(CODEC_I2C, I2C_FLAG_SB)){}

	I2C_Send7bitAddress(CODEC_I2C, CODEC_I2C_ADDRESS, I2C_Direction_Receiver);
	//wait for end of address transmission
	while (!I2C_CheckEvent(CODEC_I2C, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED)){}

	//wait until byte arrived
	while (!I2C_CheckEvent(CODEC_I2C, I2C_EVENT_MASTER_BYTE_RECEIVED)){}
	receivedByte = I2C_ReceiveData(CODEC_I2C);

	I2C_GenerateSTOP(CODEC_I2C, ENABLE);

	return receivedByte;
}
/**
  * @brief  MEMS accelerometre management of the timeout situation.
  * @param  None.
  * @retval None.
  */
uint32_t LIS302DL_TIMEOUT_UserCallback(void)
{
  /* MEMS Accelerometer Timeout error occured */
  while (1)
  {
  }
}
#ifndef USE_DEFAULT_TIMEOUT_CALLBACK
/**
  * @brief  Basic management of the timeout situation.
  * @param  None.
  * @retval None.
  */
uint32_t Codec_TIMEOUT_UserCallback(void)
{
  return (0);
}
#endif /* USE_DEFAULT_TIMEOUT_CALLBACK */
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
