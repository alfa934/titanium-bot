/*
 * motor.c
 *
 *  Created on: May 27, 2025
 *      Author: ALFA
 */

#include "motor.h"

/******************************************************************************
 * Function Definitions
 ******************************************************************************/
void Motor_Init(Motor_t *uMotor,
               GPIO_TypeDef *GPIO_A, uint16_t GPIO_PIN_A,
               GPIO_TypeDef *GPIO_B, uint16_t GPIO_PIN_B,
               TIM_HandleTypeDef *htimx, uint32_t channel,
               uint8_t reversed)
{
	uMotor -> GPIO_A 		= GPIO_A;
	uMotor -> GPIO_PIN_A 	= GPIO_PIN_A;
	uMotor -> GPIO_B 		= GPIO_B;
	uMotor -> GPIO_PIN_B 	= GPIO_PIN_B;
	uMotor -> htimx 		= htimx;
	uMotor -> channel 		= channel;
	uMotor -> reversed 		= reversed;

	HAL_TIM_PWM_Start(uMotor->htimx, uMotor->channel);

	return;
}


void Motor_Run(Motor_t *uMotor, int16_t speed)
{
    if(uMotor -> reversed)
    {
        speed = -speed;
    }

	uint8_t dir_a = (speed >= 0);
	uint8_t dir_b = (speed <  0);
	speed = abs(speed);

	HAL_GPIO_WritePin(uMotor -> GPIO_A, uMotor -> GPIO_PIN_A, dir_a);
	HAL_GPIO_WritePin(uMotor -> GPIO_B, uMotor -> GPIO_PIN_B, dir_b);

	switch (uMotor->channel)
	{
		case TIM_CHANNEL_1:
			uMotor -> htimx -> Instance -> CCR1 = speed;
			break;
		case TIM_CHANNEL_2:
			uMotor -> htimx -> Instance -> CCR2 = speed;
			break;
		case TIM_CHANNEL_3:
			uMotor -> htimx -> Instance -> CCR3 = speed;
			break;
		case TIM_CHANNEL_4:
			uMotor -> htimx -> Instance -> CCR4 = speed;
			break;
		default:
			break;
	}

	return;
}


void Encoder_Init(Encoder_t *uEncoder, TIM_HandleTypeDef *htimx, uint8_t reversed)
{
	uEncoder -> htimx 		= htimx;
	uEncoder -> reversed 	= reversed;

	HAL_TIM_Encoder_Start(uEncoder->htimx, TIM_CHANNEL_ALL);

	return;
}


void Encoder_GetCount(Encoder_t *uEncoder)
{
	if(uEncoder -> reversed)
	{
		uEncoder -> count = -(uEncoder -> htimx -> Instance -> CNT);
	}
	else
	{
		uEncoder -> count = uEncoder -> htimx -> Instance -> CNT;
	}

	return;
}


void Encoder_ResetCount(Encoder_t *uEncoder)
{
	uEncoder -> htimx -> Instance -> CNT = 0;

	return;
}


int16_t Kinematics_Triangle(Motor_e eMotor, int16_t vx, int16_t vy, int16_t vw)
{
	switch(eMotor)
	{
		case MOTOR_A:
			return vx + vw;
		case MOTOR_B:
			return (-0.5 * vx) + (0.866025 * vy) + vw;
		case MOTOR_C:
			return (-0.5 * vx) - (0.866025 * vy) + vw;

		case MOTOR_D:
		case MOTOR_E:
		case MOTOR_F:
		case MOTOR_G:
		default:
			return 0;
	}
}


int16_t Kinematics_Mecanum(Motor_e eMotor, int16_t vx, int16_t vy, int16_t vw)
{
	switch(eMotor)
	{
		case MOTOR_A:
			return 	vx + vy + vw;
		case MOTOR_B:
			return  vx - vy - vw;
		case MOTOR_C:
			return  vx - vy + vw;
		case MOTOR_D:
			return  vx + vy - vw;

		case MOTOR_E:
		case MOTOR_F:
		case MOTOR_G:
		default:
			return 0;
	}
}


int16_t Kinematics_Omni(Motor_e eMotor, int16_t vx, int16_t vy, int16_t vw)
{
	switch(eMotor)
	{
		case MOTOR_A:
			return 	vx + vy + vw;
		case MOTOR_B:
			return  vx - vy - vw;
		case MOTOR_C:
			return  vx - vy + vw;
		case MOTOR_D:
			return  vx + vy - vw;

		case MOTOR_E:
		case MOTOR_F:
		case MOTOR_G:
		default:
			return 0;
	}
}
