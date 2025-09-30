/*
 * pid.c
 *
 *  Created on: Dec 19, 2024
 *      Author: Titanium
 */

#include "pid.h"

/* Functions */
void PID_Init(PID_t *uPID, float kp, float ki, float kd, float max_output, float max_windup, float tolerance)
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

	uPID -> max_output		= max_output;
	uPID -> max_windup		= max_windup;
	uPID -> tolerance		= tolerance;
	uPID -> output			= 0;

	return;
}


void PID_Update(PID_t *uPID, float setpoint, float feedback)
{
	uPID->setpoint 		= setpoint;
	uPID->feedback 		= feedback;

	uPID->error = uPID->setpoint - uPID->feedback;

	uPID->proportional = uPID->kp * uPID->error;
	uPID->integral    += uPID->ki * uPID->error;
	uPID->derivative   = uPID->kd * (uPID->error - uPID->prev_error);
	uPID->prev_error   = uPID->error;

	if(uPID->integral >= uPID->max_windup) 			{ uPID->integral =   uPID->max_windup;  }
	else if(uPID->integral < -(uPID->max_windup)) 	{ uPID->integral = -(uPID->max_windup); }

	uPID->output = (uPID->proportional) + (uPID->integral) + (uPID->derivative);

	if(uPID->output >= uPID->max_output) 			{ uPID->output =   uPID->max_output;  }
	else if(uPID->output < -(uPID->max_output)) 	{ uPID->output = -(uPID->max_output); }

	return;
}


void PID_Update_Rotate(PID_t *uPID, float setpoint, float feedback)
{
	uPID->setpoint 		= setpoint;
	uPID->feedback 		= feedback;

	uPID->error = uPID->setpoint - uPID->feedback;

	uPID->error = fmodf(uPID->error + 180.0f, 360.0f);
	if(uPID->error < 0) uPID->error += 360.0f;
	uPID->error -= 180.0f;


	uPID->proportional = uPID->kp * uPID->error;
	uPID->integral    += uPID->ki * uPID->error;

	float error_diff = uPID->error - uPID->prev_error;
	error_diff = fmodf(error_diff + 180.0f, 360.0f);
	if (error_diff < 0) error_diff += 360.0f;
	error_diff -= 180.0f;

	uPID->derivative   = uPID->kd * error_diff;

	uPID->prev_error   = uPID->error;

	if(uPID->integral >= uPID->max_windup) 			{ uPID->integral =   uPID->max_windup;  }
	else if(uPID->integral < -(uPID->max_windup)) 	{ uPID->integral = -(uPID->max_windup); }

	uPID->output = (uPID->proportional) + (uPID->integral) + (uPID->derivative);

	if(uPID->output >= uPID->max_output) 			{ uPID->output =   uPID->max_output;  }
	else if(uPID->output < -(uPID->max_output)) 	{ uPID->output = -(uPID->max_output); }

	return;
}

void PID_Reset(PID_t *uPID)
{
	uPID->proportional = 0;
	uPID->integral = 0;
	uPID->derivative = 0;
	uPID->error = 0;
	uPID->prev_error = 0;
	uPID->output = 0;
}
