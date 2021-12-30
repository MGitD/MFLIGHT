/*
 * filter.h
 *
 *  Created on: Dec 25, 2021
 *      Author: Martin
 */

#ifndef INC_FILTER_H_
#define INC_FILTER_H_


typedef struct {

	float coeff[2];
	float out[2];

}filter;

void filter_init(filter* filt, float cutoff, float dt);
float filter_apply(filter* filt, float input);
void filter_update_cutoff(filter* filt, float cutoff, float dt);


#endif /* INC_FILTER_H_ */
