/*
 * motor.h
 *
 *  Created on: May 27, 2025
 *      Author: ALFA
 */

#ifndef INC_MOTOR_H_
#define INC_MOTOR_H_

#include "main.h"
#include "adc.h"
#include "can.h"
#include "dma.h"
#include "i2c.h"
#include "lwip.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/******************************************************************************
 * Enums
 ******************************************************************************/
typedef enum
{
	MOTOR_A,
	MOTOR_B,
	MOTOR_C,
	MOTOR_D,
	MOTOR_E,
	MOTOR_F,
	MOTOR_G
} Motor_e;

/******************************************************************************
 * Structs
 ******************************************************************************/
typedef struct
{
    GPIO_TypeDef* 		GPIO_A;
    uint16_t 			GPIO_PIN_A;
    GPIO_TypeDef* 		GPIO_B;
    uint16_t 			GPIO_PIN_B;

    TIM_HandleTypeDef* 	htimx;
    uint32_t 			channel;

    uint8_t 			reversed;

} Motor_t;


typedef struct
{
    TIM_HandleTypeDef* 	htimx;
    int16_t 			count;
    uint8_t 			reversed;

} Encoder_t;


/******************************************************************************
 * Function Prototypes
 ******************************************************************************/
void 	Motor_Init(	Motor_t *uMotor,
                	GPIO_TypeDef *GPIO_A, uint16_t GPIO_PIN_A,
					GPIO_TypeDef *GPIO_B, uint16_t GPIO_PIN_B,
					TIM_HandleTypeDef *htimx, uint32_t channel,
					uint8_t reversed);

void 	Motor_Run(Motor_t *uMotor, int16_t speed);

void 	Encoder_Init(Encoder_t *uEncoder, TIM_HandleTypeDef *htimx, uint8_t reversed);

void 	Encoder_GetCount(Encoder_t *uEncoder);

void 	Encoder_ResetCount(Encoder_t *uEncoder);

int16_t Kinematics_Triangle(Motor_e eMotor, int16_t vx, int16_t vy, int16_t vw);

int16_t Kinematics_Mecanum(Motor_e eMotor, int16_t vx, int16_t vy, int16_t vw);

int16_t Kinematics_Omni(Motor_e eMotor, int16_t vx, int16_t vy, int16_t vw);

//  - TIM5_CH1 - PA0
//  - TIM5_CH4 - PA3
//  - TIM8_CH3 - PC8
//  - TIM9_CH1 - PE5
//  - TIM9_CH2 - PE6
//  - TIM10_CH1 - PB8
//  - TIM11_CH1 - PB9
//  - TIM12_CH1 - PB14
//  - TIM12_CH2 - PB15
//  - TIM13_CH1 - PA6

#endif /* INC_MOTOR_H_ */
