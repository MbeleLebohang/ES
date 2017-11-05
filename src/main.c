/**
  ******************************************************************************
  * @file    Audio_playback_and_record/src/main.c 
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    28-October-2011
  * @brief   Main program body
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  */ 

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/** @addtogroup STM32F4-Discovery_Audio_Player_Recorder
  * @{
  */ 

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define MEDIA_IntFLASH
/* Private variables ---------------------------------------------------------*/
#if defined MEDIA_USB_KEY
 USB_OTG_CORE_HANDLE          USB_OTG_Core;
 USBH_HOST                    USB_Host;
#endif

RCC_ClocksTypeDef RCC_Clocks;
__IO uint8_t RepeatState = 0;
__IO uint16_t CCR_Val = 16826;
extern __IO uint8_t LED_Toggle;

/* Private function prototypes -----------------------------------------------*/

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Main program.
  * @param  None
  * @retval None
*/
int main(void)
{ 
  /* Initialise LEDS */
  STM_EVAL_LEDInit(LED4);
 
  /* Green Led On: start of application */
  STM_EVAL_LEDOn(LED4);
       
  /* SysTick end of count event each 10ms */
  RCC_GetClocksFreq(&RCC_Clocks);
  SysTick_Config(RCC_Clocks.HCLK_Frequency / 100);
  
  /* Configure TIM4 Peripheral to manage LEDs lighting */
  //TIM_LED_Config();
  
  /* Initialize the repeat status */
  RepeatState = 0;
  LED_Toggle = 7;
  
  STM_EVAL_PBInit(BUTTON_USER, BUTTON_MODE_GPIO);
  WavePlayBack(I2S_AudioFreq_48k); 

  int vol = 70;
  while (1){
	  if(STM_EVAL_PBGetState(BUTTON_USER)){
		  while(STM_EVAL_PBGetState(BUTTON_USER)){}

		  WaveplayerCtrlVolume(vol++);
		  if(vol > 100){
			  vol = 0;
		  }
	  }
  }
  
}


#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *   where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

  
/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
