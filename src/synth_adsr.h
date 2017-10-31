/*
 * synth_adsr.h
 *
 *  Created on: 26 Oct 2017
 *      Author: MBELE LEBOHANG
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef SYNTH_ADSR_H_
#define SYNTH_ADSR_H_

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"


#define MAX_ADSR_AMPLITUDE     		65535
#define OVERSHOT_ADSR_AMPLITUDE 	65980	/* 0.679% overshot */



typedef enum
{
	IDLE = 0,
	ATTACK = 1,
	DECAY = 2,
	SUSTAIN = 3,
	RELEASE = 4,

} ADSR_FunctionalState;

typedef struct{
	uint8_t ADSR_AttackTimeConstant;			/* 0-255 */

	uint8_t ADSR_DecayTimeConstant;				/* 0-255 */

	uint8_t ADSR_SustainLevel;					/* 0-65535 */

	uint16_t ADSR_ReleaseTimeConstant;			/* 0-65535 */

	float	ADSR_Output;

	ADSR_FunctionalState ADSR_State;

	FunctionalState	ADSR_Gate;
}ADSR_TypeDef;



float ADSR_Process(ADSR_TypeDef* ADSR);

void ADSR_UpdateCoefficients(ADSR_TypeDef* ADSR, ADSR_FunctionalState State, float Rate);


#endif /* SYNTH_ADSR_H_ */
