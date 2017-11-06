/**
  ******************************************************************************
  * @file    stm32f4_discovery.h
  * @author  MCD Application Team
  * @version V1.1.1
  * @date    14-May-2012
  * @brief   This file contains definitions for STM32F4-Discovery Kit's Leds and
  *          push-button hardware resources.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2012 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software
  * distributed under the License is distributed on an "AS IS" BASIS,
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STM32F4_MAIN_H
#define __STM32F4_MAIN_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
 #include "stm32f4xx.h"

/** @addtogroup Utilities
  * @{
  */

/** @addtogroup STM32F4_DISCOVERY
  * @{
  */

/** @addtogroup STM32F4_DISCOVERY_LOW_LEVEL
  * @{
  */

/** @defgroup STM32F4_DISCOVERY_LOW_LEVEL_Exported_Types
  * @{
  */

/**
  * @}
  */

/** @defgroup STM32F4_DISCOVERY_LOW_LEVEL_Exported_Constants
  * @{
  */


/**
  * @}
  */

/** @defgroup STM32F4_DISCOVERY_LOW_LEVEL_Exported_Macros
  * @{
  */
/************** Definition for USARTx resources *********************/
#define USARTx_TX_PIN                    GPIO_Pin_6
#define USARTx_TX_SOURCE                 GPIO_PinSource6
#define USARTx_TX_AF                     GPIO_AF_USART6

#define USARTx_RX_PIN                    GPIO_Pin_7
#define USARTx_RX_SOURCE                 GPIO_PinSource7
#define USARTx_RX_AF                     GPIO_AF_USART6

/****************** Definition for CODEC resources ***************************/
#define CODEC_RESET_PIN   GPIO_Pin_4      	//port D

#define I2C_SCL_PIN		GPIO_Pin_6  		//port B
#define I2C_SDA_PIN		GPIO_Pin_9  		//port B

#define I2S3_MCLK_PIN 	GPIO_Pin_7   		//port C
#define I2S3_SCLK_PIN 	GPIO_Pin_10  		//port C
#define I2S3_SD_PIN 	GPIO_Pin_12  		//port C

#define I2S3_WS_PIN 	GPIO_Pin_4   		//port A

#define CORE_I2C_ADDRESS 0x33
#define CODEC_I2C_ADDRESS 0x94

#define CODEC_MAPBYTE_INC 0x80

 /*******register map bytes for CS43L22******/
 #define CODEC_MAP_CHIP_ID 0x01
 #define CODEC_MAP_PWR_CTRL1 0x02
 #define CODEC_MAP_PWR_CTRL2 0x04
 #define CODEC_MAP_CLK_CTRL  0x05
 #define CODEC_MAP_IF_CTRL1  0x06
 #define CODEC_MAP_IF_CTRL2  0x07
 #define CODEC_MAP_PASSTHROUGH_A_SELECT 0x08
 #define CODEC_MAP_PASSTHROUGH_B_SELECT 0x09
 #define CODEC_MAP_ANALOG_SET 0x0A
 #define CODEC_MAP_PASSTHROUGH_GANG_CTRL 0x0C
 #define CODEC_MAP_PLAYBACK_CTRL1 0x0D
 #define CODEC_MAP_MISC_CTRL 0x0E
 #define CODEC_MAP_PLAYBACK_CTRL2 0x0F
 #define CODEC_MAP_PASSTHROUGH_A_VOL 0x14
 #define CODEC_MAP_PASSTHROUGH_B_VOL 0x15
 #define CODEC_MAP_PCMA_VOL 0x1A
 #define CODEC_MAP_PCMB_VOL 0x1B
 #define CODEC_MAP_BEEP_FREQ_ONTIME 0x1C
 #define CODEC_MAP_BEEP_VOL_OFFTIME 0x1D
 #define CODEC_MAP_BEEP_TONE_CFG 0x1E
 #define CODEC_MAP_TONE_CTRL 0x1F
 #define CODEC_MAP_MASTER_A_VOL 0x20
 #define CODEC_MAP_MASTER_B_VOL 0x21
 #define CODEC_MAP_HP_A_VOL 0x22
 #define CODEC_MAP_HP_B_VOL 0x23
 #define CODEC_MAP_SPEAK_A_VOL 0x24
 #define CODEC_MAP_SPEAK_B_VOL 0x25
 #define CODEC_MAP_CH_MIX_SWAP 0x26
 #define CODEC_MAP_LIMIT_CTRL1 0x27
 #define CODEC_MAP_LIMIT_CTRL2 0x28
 #define CODEC_MAP_LIMIT_ATTACK 0x29
 #define CODEC_MAP_OVFL_CLK_STATUS 0x2E
 #define CODEC_MAP_BATT_COMP 0x2F
 #define CODEC_MAP_VP_BATT_LEVEL 0x30
 #define CODEC_MAP_SPEAK_STATUS 0x31
 #define CODEC_MAP_CHARGE_PUMP_FREQ 0x34

/**
  * @}
  */


/** @defgroup STM32F4_DISCOVERY_LOW_LEVEL_Exported_Functions
  * @{
  */

void GPIO_Configuration(void);
void ADC_Configuration(void);
void RCC_Configuration(void);
void DAC_Configuration(void);
void TIM_Configuration(uint16_t period, uint16_t prescaler);

void NVIC_Configuration(void);
void USARTx_Configuration(void);

void CODEC_Configuration(void);
uint8_t CODEC_Volume_CTRL(uint8_t vol);
void Send_CODEC_Command(uint8_t[], uint8_t);
void CS43L22_Configuration(void);
uint8_t Read_CODEC_Register(uint8_t);

/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif /* __STM32F4_DISCOVERY_H */
/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */



/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
