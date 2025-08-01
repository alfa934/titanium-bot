/*
 * pid.c
 *
 *  Created on: Dec 19, 2024
 *      Author: Titanium
 */

#include "pid.h"

/* Functions */
void PID_Init(PID_t *uPID, float kp, float ki, float kd)
{
	uPID -> kp = kp;
	uPID -> ki = ki;
	uPID -> kd = kd;

	uPID -> proportional 	= 0;
	uPID -> integral 		= 0;
	uPID -> derivative 		= 0;

	uPID -> error 			= 0;
	uPID -> prev_error 		= 0;
	uPID -> setpoint 		= 0;
	uPID -> feedback		= 0;

	uPID -> max_output		= 0;
	uPID -> output			= 0;

	return;
}


void PID_Update(PID_t *uPID, float setpoint, float feedback, float max_output)
{
	uPID->setpoint 		= setpoint;
	uPID->feedback 		= feedback;
	uPID->max_output 	= max_output;

	uPID->error = uPID->setpoint - uPID->feedback;

//	if(uPID->mode == ANGULAR_PID)
//	{
//	    if(uPID->error > 180) 			{ setpoint -= 360; }
//	    else if(uPID->error < -180) 	{ setpoint += 360; }
//	    uPID->error = setpoint - feedback;
//	}

	uPID->proportional = uPID->kp * uPID->error;
	uPID->integral    += uPID->ki * uPID->error;
	uPID->derivative   = uPID->kd * (uPID->error - uPID->prev_error);
	uPID->prev_error   = uPID->error;

	if(uPID->integral >= uPID->max_output) 			{ uPID->integral =   uPID->max_output; }
	else if(uPID->integral < -(uPID->max_output)) 	{ uPID->integral = -(uPID->max_output); }

	uPID->output = (uPID->proportional) + (uPID->integral) + (uPID->derivative);

	if(uPID->output >= uPID->max_output) 			{ uPID->output =   uPID->max_output; }
	else if(uPID->output < -(uPID->max_output)) 	{ uPID->output = -(uPID->max_output); }

	return;
}
