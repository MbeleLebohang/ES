/*
 * synth_adsr.h
 *
 *  Created on: 26 Oct 2017
 *      Author: MBELE
 */

#ifndef SYNTH_ADSR_H_
#define SYNTH_ADSR_H_

#define FULL_VOLUME     31 	/* 5-bit volumes */

// Default envelope values (slightly percussive)
// Play around with these!

typedef struct{
	uint8_t ADSR_AttackRate;			/* 0-255 */

	uint8_t ADSR_DecayRate;				/* 0-255 */

	uint8_t ADSR_SustainLevel;			/* 0-255 */

	uint16_t ADSR_ReleaseRate;			/* 0-65535 */

	float	ADSR_Output;

	ADSR_FunctionalState ADSR_State;

	FunctionalState	ADSR_Gate;
}ADSR_TypeDef;

typedef enum
{
	IDLE = 0,
	ATTACK = 1,
	DECAY = 2,
	SUSTAIN = 3,
	RELEASE = 4,

} ADSR_FunctionalState;

float ADSR_Process(ADSR_TypeDef* ADSR){

	switch(ADSR->ADSR_State){
		case IDLE:
			// Do nothing
			break;
		case ATTACK:
			// Attack state
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

		default:
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

		default:
			// Something went wrong
	}
}


#endif /* SYNTH_ADSR_H_ */
