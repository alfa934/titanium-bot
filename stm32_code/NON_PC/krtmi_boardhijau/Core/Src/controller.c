/*
 * controller.c
 *
 *  Created on: May 27, 2025
 *      Author: ALFA
 */

#include "controller.h"

int8_t Controller_Drift(int8_t value, int8_t max)
{
	if(abs(value) < max)
	{
		return 0;
	}
	else if(value > 0)
	{
		return value - max;
	}
	else
	{
		return value + max;
	}
}

int32_t map(int32_t x, int32_t in_min, int32_t in_max, int32_t out_min, int32_t out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

