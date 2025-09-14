/*
 * controller.h
 *
 *  Created on: May 27, 2025
 *      Author: ALFA
 */

#ifndef INC_CONTROLLER_H_
#define INC_CONTROLLER_H_

#include "main.h"

typedef struct
{
	int8_t rX;
	int8_t rY;
	int8_t lX;
	int8_t lY;
	uint8_t r2;
	uint8_t l2;
	uint8_t r1;
	uint8_t l1;
	uint8_t r3;
	uint8_t l3;
	uint8_t crs;
	uint8_t sqr;
	uint8_t tri;
	uint8_t cir;
	uint8_t up;
	uint8_t down;
	uint8_t right;
	uint8_t left;
	uint8_t share;
	uint8_t option;

	uint8_t ps;
	uint8_t touchpad;
	uint8_t battery;

	int16_t gX, gY, gZ;
	int16_t aX, aY, aZ;

} Controller_t ;

int8_t Controller_Drift(int8_t value, int8_t max);
int32_t map(int32_t x, int32_t in_min, int32_t in_max, int32_t out_min, int32_t out_max);


#endif /* INC_CONTROLLER_H_ */
