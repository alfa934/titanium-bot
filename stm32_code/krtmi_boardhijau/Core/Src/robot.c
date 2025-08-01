/*
 * robot.c
 *
 *  Created on: Jun 24, 2025
 *      Author: ALFA
 */

#include "robot.h"

Controller_t input;

Motor_t motorA;
Motor_t motorB;
Motor_t motorC;

Encoder_t encA;
Encoder_t encB;
Encoder_t encC;
Encoder_t encX;
Encoder_t encY;

PID_t PID_A;
PID_t PID_B;
PID_t PID_C;
PID_t PID_VY;
PID_t PID_VW;

int16_t vx;
int16_t vy;
int16_t vw;
int16_t vx_controller;
int16_t vy_controller;
int16_t vw_controller;

float kp;
float ki;
float kd;

char UART2_RX[23];
char UART3_RX[43];
char UART4_RX[53];
char UART5_RX[23];

float yaw_degree;
float yaw_radian;
float yaw_adjust;
uint16_t UltraSonic[4];


void Robot_Init()
{
	Motor_Init(	&motorA,
				GPIOD, GPIO_PIN_11,
				GPIOE, GPIO_PIN_14,
				&htim12, TIM_CHANNEL_2, 0);

	Motor_Init(	&motorB,
				GPIOD, GPIO_PIN_10,
				GPIOD, GPIO_PIN_8,
				&htim12, TIM_CHANNEL_1, 0);

	Motor_Init(	&motorC,
				GPIOE, GPIO_PIN_3,
				GPIOE, GPIO_PIN_0,
				&htim10, TIM_CHANNEL_1, 0);

	Encoder_Init(&encA, &htim2, 0);
	Encoder_Init(&encB, &htim4, 1);
	Encoder_Init(&encC, &htim1, 1);

	kp = 45;
	ki = 2.5;
	kd = 0;

    PID_Init(&PID_A, kp, ki, kd);
    PID_Init(&PID_B, kp, ki, kd);
    PID_Init(&PID_C, kp, ki, kd);
    PID_Init(&PID_VY, 0.65, 0, 0);
    PID_Init(&PID_VW, 0.5, 0, 0);

    HAL_UART_Receive_DMA(&huart2, (uint8_t*)UART2_RX, sizeof(UART2_RX));
    HAL_UART_Receive_DMA(&huart3, (uint8_t*)UART3_RX, sizeof(UART3_RX));
    HAL_UART_Receive_DMA(&huart4, (uint8_t*)UART4_RX, sizeof(UART4_RX));
    HAL_UART_Receive_DMA(&huart5, (uint8_t*)UART5_RX, sizeof(UART5_RX));

	udpClient_connect();

	HAL_TIM_Base_Start_IT(&htim6);
}

void Robot_Motor()
{
	static uint16_t timer = 0;

	if(timer >= 9)
	{
//		float avg_distance = (UltraSonic[0] + UltraSonic[1]) / 2;
//
//		PID_Update(&PID_VY, 20, avg_distance, 5);
//
//		PID_Update(&PID_VW, UltraSonic[0], UltraSonic[1], 5);
//
//		int16_t va = Kinematics_Triangle(MOTOR_A, vx, vy, vw);
//		int16_t vb = Kinematics_Triangle(MOTOR_B, vx, vy, vw);
//		int16_t vc = Kinematics_Triangle(MOTOR_C, vx, vy, vw);

		int16_t va = udp_rx.motor_a;
		int16_t vb = udp_rx.motor_b;
		int16_t vc = udp_rx.motor_c;

		Encoder_GetCount(&encA);
		Encoder_GetCount(&encB);
		Encoder_GetCount(&encC);

		/* Save UDP */
		udp_tx.enc_a = encA.count;
		udp_tx.enc_b = encB.count;
		udp_tx.enc_c = encC.count;

		Encoder_ResetCount(&encA);
		Encoder_ResetCount(&encB);
		Encoder_ResetCount(&encC);

		PID_Update(&PID_A, (float)va, (float)encA.count, 999);
		PID_Update(&PID_B, (float)vb, (float)encB.count, 999);
		PID_Update(&PID_C, (float)vc, (float)encC.count, 999);

		Motor_Run(&motorA, (int16_t)PID_A.output);
		Motor_Run(&motorB, (int16_t)PID_B.output);
		Motor_Run(&motorC, (int16_t)PID_C.output);

		timer = 0;
	}

	timer++;
}

void Robot_LED_Blink()
{
	static uint16_t timer = 0;

	if(timer >= 99)
	{
		HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_1);
		timer = 0;
	}

	timer++;
}

void Robot_Loop()
{
	Robot_Motor();

	Robot_LED_Blink();
}


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart == &huart2)
	{
		memcpy(&yaw_degree, UART2_RX + 3, 4);

		yaw_radian = (yaw_degree - yaw_adjust) * M_PI/180.0;

		/* Save UDP */
		udp_tx.yaw_degree = yaw_degree;

		HAL_UART_Receive_DMA(&huart2, (uint8_t*)UART2_RX, sizeof(UART2_RX));
	}

	if(huart == &huart3)
	{
		memcpy((uint8_t*)&input + 4, UART3_RX + 3, sizeof(input) - 4);

		input.lX = Controller_Drift(input.lX_raw, 12);
		input.lY = Controller_Drift(input.lY_raw, 12);
		input.rX = Controller_Drift(input.rX_raw, 12);
		input.rY = Controller_Drift(input.rY_raw, 12);

		input.lX = map(input.lX, -128, 127, -30, 30);
		input.lY = map(input.lY, -128, 127, -30, 30);
		input.rX = map(input.rX, -128, 127, -15, 15);
		input.rY = map(input.rY, -128, 127, -30, 30);

		//--- Robot Centric
//		vx_controller = input.lX;
//		vy_controller = input.lY;
//		vw_controller = -(input.rX);

		//--- Field Centric
//		vx_controller = input.lX *  cos(yaw_radian) + input.lY * sin(yaw_radian);
//		vy_controller = input.lX * -sin(yaw_radian) + input.lY * cos(yaw_radian);
//		vw_controller = -(input.rX);
//
//		vx = vx_controller;
//		vy = vy_controller;
//		vw = vw_controller;

		if(input.crs)
		{
			yaw_adjust = yaw_degree;
		}


		HAL_UART_Receive_DMA(&huart3, (uint8_t*)UART3_RX, sizeof(UART3_RX));
	}

	if(huart == &huart4)
	{
		memcpy(&encX.count, UART4_RX + 3, 2);
		memcpy(&encY.count, UART4_RX + 5, 2);

		/* Save UDP */
		memcpy(&udp_tx.enc_x, &encX.count, 2);
		memcpy(&udp_tx.enc_y, &encY.count, 2);

		HAL_UART_Receive_DMA(&huart4, (uint8_t*)UART4_RX, sizeof(UART4_RX));
	}

	if(huart == &huart5)
	{
		memcpy(UltraSonic, UART5_RX + 3, sizeof(UltraSonic));

		/* Save UDP */
		memcpy(udp_tx.ultrasonic, UltraSonic, sizeof(udp_tx.ultrasonic));

		HAL_UART_Receive_DMA(&huart5, (uint8_t*)UART5_RX, sizeof(UART5_RX));
	}

}

void HAL_UART_RxHalfCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart == &huart2)
	{
		if(!(UART2_RX[0] == 'A' && UART2_RX[1] == 'B' && UART2_RX[2] == 'C'))
		{
			HAL_UART_AbortReceive(&huart2);
			HAL_UART_Receive_DMA(&huart2, (uint8_t*)UART2_RX, sizeof(UART2_RX));
		}
	}

	if(huart == &huart3)
	{
		if(!(UART3_RX[0] == 'A' && UART3_RX[1] == 'B' && UART3_RX[2] == 'C'))
		{
			HAL_UART_AbortReceive(&huart3);
			HAL_UART_Receive_DMA(&huart3, (uint8_t*)UART3_RX, sizeof(UART3_RX));
		}
	}

	if(huart == &huart4)
	{
		if(!(UART4_RX[0] == 'A' && UART4_RX[1] == 'B' && UART4_RX[2] == 'C'))
		{
			HAL_UART_AbortReceive(&huart4);
			HAL_UART_Receive_DMA(&huart4, (uint8_t*)UART4_RX, sizeof(UART4_RX));
		}
	}

	if(huart == &huart5)
	{
		if(!(UART5_RX[0] == 'A' && UART5_RX[1] == 'B' && UART5_RX[2] == 'C'))
		{
			HAL_UART_AbortReceive(&huart5);
			HAL_UART_Receive_DMA(&huart5, (uint8_t*)UART5_RX, sizeof(UART5_RX));
		}
	}


}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
	if(huart == &huart2)
	{
		HAL_UART_AbortReceive(&huart2);
		HAL_UART_Receive_DMA(&huart2, (uint8_t*)UART2_RX, sizeof(UART2_RX));
	}

	if(huart == &huart3)
	{
		HAL_UART_AbortReceive(&huart3);
		HAL_UART_Receive_DMA(&huart3, (uint8_t*)UART3_RX, sizeof(UART3_RX));
	}

	if(huart == &huart4)
	{
		HAL_UART_AbortReceive(&huart4);
		HAL_UART_Receive_DMA(&huart4, (uint8_t*)UART4_RX, sizeof(UART4_RX));
	}

	if(huart == &huart5)
	{
		HAL_UART_AbortReceive(&huart5);
		HAL_UART_Receive_DMA(&huart5, (uint8_t*)UART5_RX, sizeof(UART5_RX));
	}

}
