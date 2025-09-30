/*
 * pid.h
 *
 *  Created on: Dec 19, 2024
 *      Author: ASUS
 */

#ifndef INC_PID_H_
#define INC_PID_H_

#include "main.h"


/* Structs */
typedef struct
{
	float   kp;
	float   ki;
	float   kd;

	float 	proportional;
	float 	integral;
	float 	derivative;

	float 	error;
	float 	prev_error;
	float	setpoint;
	float 	feedback;

	float 	max_output;
	float 	max_windup;
	float	tolerance;
	float 	output;

} PID_t;


/* Functions */
void PID_Init(PID_t *uPID, float kp, float ki, float kd, float max_output, float max_windup, float tolerance);
void PID_Update(PID_t *uPID, float setpoint, float feedback);
void PID_Update_Rotate(PID_t *uPID, float setpoint, float feedback);
void PID_Reset(PID_t *uPID);

#endif /* INC_PID_H_ */
