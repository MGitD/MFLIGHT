/*
 * map.c
 *
 *  Created on: Dec 26, 2021
 *      Author: Martin
 */


#include "map.h"



float map(float value, float MIN_DES, float MAX_DES, float MIN_IN, float MAX_IN){
float out;
float slope = (MAX_DES-MIN_DES) / (MAX_IN - MIN_IN);
out = slope * (value - MIN_IN) + MIN_DES;
return (out);
}


float constrain(float value, float MIN, float MAX){
	float out = value;
if (out < MIN){
	out = MIN;
}
if (out > MAX){
	out = MAX;
}
return out;
}
