/*
 * filter.c
 *
 *  Created on: Dec 25, 2021
 *      Author: Martin
 */

#include "filter.h"

// initialise filter with cutoff and dt
void filter_init(filter* filt, float cutoff, float dt){

	float RC = 1.0f / (6.28318530718f * cutoff);
	filt->coeff[0] = dt / (RC + dt);
	filt->coeff[1] = RC / (RC + dt);
	filt->out[0] = 0.0f;
	filt->out[1] = 0.0f;
}


// apply filter on sample
float filter_apply(filter* filt, float input){

	// Note: current out = out[0]
	filt->out[1] = filt->out[0];
	filt->out[0] = filt->coeff[0] * input + filt->coeff[1] * filt->out[1];

	return filt->out[0];

}

void filter_update_cutoff(filter* filt, float cutoff, float dt){

	float RC = 1.0f / (6.28318530718f * cutoff);
	filt->coeff[0] = dt / (RC + dt);
	filt->coeff[1] = RC / (RC + dt);
}


