/*
 * synth_adsr.c
 *
 *  Created on: 26 October 2017
 *      Author: MBELE
 */

#include <math.h>
#include "synth_adsr.h"


double t = 0;


float ADSR_Process(ADSR_TypeDef* ADSR){

	switch(ADSR->ADSR_State){
		case IDLE:
			// Do nothing
			break;
		case ATTACK:
			// Attack state
			ADSR->ADSR_Output = OVERSHOT_ADSR_AMPLITUDE*(1-exp(-t/(ADSR->ADSR_AttackTimeConstant)));
			t++;

			if(ADSR->ADSR_Output >= MAX_ADSR_AMPLITUDE){
				t = 0;
				ADSR->ADSR_State = IDLE;
			}
			break;

		case DECAY:
			// Decay state
			break;

		case SUSTAIN:
			// Sustain state
			break;

		case RELEASE:
			// Released state
			break;

			// Something went wrong
	}

	return ADSR->ADSR_Output;
}

void ADSR_UpdateCoefficients(ADSR_TypeDef* ADSR, ADSR_FunctionalState State, float Rate){

	switch(ADSR->ADSR_State){
		case IDLE:
			// Do nothing
			break;
		case ATTACK:
			// Update Attack properties
			break;

		case DECAY:
			// Update Decay properties
			break;

		case SUSTAIN:
			// Update Sustain properties
			break;

		case RELEASE:
			// Update Released Release
			break;
			// Something went wrong
	}
}
