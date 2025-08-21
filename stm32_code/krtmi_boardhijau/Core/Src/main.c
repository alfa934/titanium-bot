/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "can.h"
#include "dma.h"
#include "i2c.h"
#include "lwip.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "udpClientRAW.h"
#include "controller.h"
#include "motor.h"
#include "pid.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
float Y_ENC_PULSE_CM = 0.0488758553274682;
float X_ENC_PULSE_CM = 0.0539374325782093;

uint8_t udp_cnt = 0;

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

char UART1_RX_BUFFER[53];
char UART2_RX_BUFFER[23];
char UART3_RX_BUFFER[43];
char UART4_RX_BUFFER[53];
char UART5_RX_BUFFER[23];
char UART6_RX_BUFFER[7];

char UART1_TX_BUFFER[53] = "ABC";
int16_t cnt_rx = 0;
int16_t cnt_tx = 0;


float yaw_degree;
float yaw_radian;
float yaw_adjust;
uint16_t UltraSonic[4];


void Robot_Init()
{
	Motor_Init(	&motorA,
				GPIOC, GPIO_PIN_15,
				GPIOC, GPIO_PIN_13,
				&htim11, TIM_CHANNEL_1, 1);

	Motor_Init(	&motorB,
				GPIOE, GPIO_PIN_3,
				GPIOE, GPIO_PIN_0,
				&htim10, TIM_CHANNEL_1, 0);

	Motor_Init(	&motorC,
				GPIOD, GPIO_PIN_10,
				GPIOD, GPIO_PIN_8,
				&htim12, TIM_CHANNEL_1, 1);

	Encoder_Init(&encA, &htim3, 0);
	Encoder_Init(&encB, &htim4, 1);
	Encoder_Init(&encC, &htim2, 0);

	kp = 45;
	ki = 2.5;
	kd = 0;

    PID_Init(&PID_A, kp, ki, kd);
    PID_Init(&PID_B, kp, ki, kd);
    PID_Init(&PID_C, kp, ki, kd);
    PID_Init(&PID_VY, 0.65, 0, 0);
    PID_Init(&PID_VW, 0.5, 0, 0);

    HAL_UART_Receive_DMA(&huart1, (uint8_t*)UART1_RX_BUFFER, sizeof(UART1_RX_BUFFER));
    HAL_UART_Receive_DMA(&huart2, (uint8_t*)UART2_RX_BUFFER, sizeof(UART2_RX_BUFFER));
    HAL_UART_Receive_DMA(&huart3, (uint8_t*)UART3_RX_BUFFER, sizeof(UART3_RX_BUFFER));
    HAL_UART_Receive_DMA(&huart4, (uint8_t*)UART4_RX_BUFFER, sizeof(UART4_RX_BUFFER));
    HAL_UART_Receive_DMA(&huart5, (uint8_t*)UART5_RX_BUFFER, sizeof(UART5_RX_BUFFER));
    HAL_UART_Receive_DMA(&huart6, (uint8_t*)UART6_RX_BUFFER, sizeof(UART6_RX_BUFFER));

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

		vx = 3;
		vy = 0;
		vw = 0;

		int16_t va = Kinematics_Triangle(MOTOR_A, vx, vy, vw);
		int16_t vb = Kinematics_Triangle(MOTOR_B, vx, vy, vw);
		int16_t vc = Kinematics_Triangle(MOTOR_C, vx, vy, vw);

//		int16_t va = udp_rx.motor_a;
//		int16_t vb = udp_rx.motor_b;
//		int16_t vc = udp_rx.motor_c;


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
	static uint8_t state = 0;

	if(timer >= 99)
	{
		state = !(state);
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_1, state);
		timer = 0;
	}

	timer++;
}

void Robot_Transmit_UART()
{
	memcpy(UART1_TX_BUFFER + 3, &cnt_tx, 2);
	HAL_UART_Transmit_DMA(&huart1, (uint8_t*)UART1_TX_BUFFER, sizeof(UART1_TX_BUFFER));
	cnt_tx++;
}

void Robot_Loop()
{
	Robot_Transmit_UART();

	Robot_Motor();

	Robot_LED_Blink();
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{

	if(htim == &htim6)
	{
		udp_cnt++;

		Robot_Loop();
	}
}


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart == &huart1) //--- VGT ARM
	{
		memcpy(&cnt_rx, UART1_RX_BUFFER + 3, 2);

		HAL_UART_Receive_DMA(&huart1, (uint8_t*)UART1_RX_BUFFER, sizeof(UART1_RX_BUFFER));
	}

//	if(huart == &huart2)
//	{
//		memcpy(&yaw_degree, UART2_RX_BUFFER + 3, 4);
//
//		yaw_radian = (yaw_degree - yaw_adjust) * M_PI/180.0;
//
//		/* Save UDP */
//		udp_tx.yaw_degree = yaw_degree;
//
//		HAL_UART_Receive_DMA(&huart2, (uint8_t*)UART2_RX_BUFFER, sizeof(UART2_RX_BUFFER));
//	}

	if(huart == &huart3)
	{
		memcpy((uint8_t*)&input + 4, UART3_RX_BUFFER + 3, sizeof(input) - 4);

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


		HAL_UART_Receive_DMA(&huart3, (uint8_t*)UART3_RX_BUFFER, sizeof(UART3_RX_BUFFER));
	}

	if(huart == &huart4)
	{
		memcpy(&encX.count, UART4_RX_BUFFER + 3, 2);
		memcpy(&encY.count, UART4_RX_BUFFER + 5, 2);

		/* Save UDP */
		memcpy(&udp_tx.enc_x, &encX.count, 2);
		memcpy(&udp_tx.enc_y, &encY.count, 2);

		HAL_UART_Receive_DMA(&huart4, (uint8_t*)UART4_RX_BUFFER, sizeof(UART4_RX_BUFFER));
	}

	if(huart == &huart5)
	{
		memcpy(UltraSonic, UART5_RX_BUFFER + 3, sizeof(UltraSonic));

		/* Save UDP */
		memcpy(udp_tx.ultrasonic, UltraSonic, sizeof(udp_tx.ultrasonic));

		HAL_UART_Receive_DMA(&huart5, (uint8_t*)UART5_RX_BUFFER, sizeof(UART5_RX_BUFFER));
	}

	if(huart == &huart6)
	{
		memcpy(&yaw_degree, UART6_RX_BUFFER + 3, 4);

		yaw_radian = (yaw_degree - yaw_adjust) * M_PI/180.0;

		/* Save UDP */
		udp_tx.yaw_degree = yaw_degree;

		HAL_UART_Receive_DMA(&huart6, (uint8_t*)UART6_RX_BUFFER, sizeof(UART6_RX_BUFFER));
	}

}

void HAL_UART_RxHalfCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart == &huart1) //--- VGT ARM
	{
		if(!(UART1_RX_BUFFER[0] == 'A' && UART1_RX_BUFFER[1] == 'B' && UART1_RX_BUFFER[2] == 'C'))
		{
			HAL_UART_AbortReceive(&huart1);
			HAL_UART_Receive_DMA(&huart1, (uint8_t*)UART1_RX_BUFFER, sizeof(UART1_RX_BUFFER));
		}
	}

	if(huart == &huart2)
	{
		if(!(UART2_RX_BUFFER[0] == 'A' && UART2_RX_BUFFER[1] == 'B' && UART2_RX_BUFFER[2] == 'C'))
		{
			HAL_UART_AbortReceive(&huart2);
			HAL_UART_Receive_DMA(&huart2, (uint8_t*)UART2_RX_BUFFER, sizeof(UART2_RX_BUFFER));
		}
	}

	if(huart == &huart3)
	{
		if(!(UART3_RX_BUFFER[0] == 'A' && UART3_RX_BUFFER[1] == 'B' && UART3_RX_BUFFER[2] == 'C'))
		{
			HAL_UART_AbortReceive(&huart3);
			HAL_UART_Receive_DMA(&huart3, (uint8_t*)UART3_RX_BUFFER, sizeof(UART3_RX_BUFFER));
		}
	}

	if(huart == &huart4)
	{
		if(!(UART4_RX_BUFFER[0] == 'A' && UART4_RX_BUFFER[1] == 'B' && UART4_RX_BUFFER[2] == 'C'))
		{
			HAL_UART_AbortReceive(&huart4);
			HAL_UART_Receive_DMA(&huart4, (uint8_t*)UART4_RX_BUFFER, sizeof(UART4_RX_BUFFER));
		}
	}

	if(huart == &huart5)
	{
		if(!(UART5_RX_BUFFER[0] == 'A' && UART5_RX_BUFFER[1] == 'B' && UART5_RX_BUFFER[2] == 'C'))
		{
			HAL_UART_AbortReceive(&huart5);
			HAL_UART_Receive_DMA(&huart5, (uint8_t*)UART5_RX_BUFFER, sizeof(UART5_RX_BUFFER));
		}
	}

	if(huart == &huart6)
	{
		if(!(UART6_RX_BUFFER[0] == 'A' && UART6_RX_BUFFER[1] == 'B' && UART6_RX_BUFFER[2] == 'C'))
		{
			HAL_UART_AbortReceive(&huart6);
			HAL_UART_Receive_DMA(&huart6, (uint8_t*)UART6_RX_BUFFER, sizeof(UART6_RX_BUFFER));
		}
	}

}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
	if(huart == &huart1) //--- VGT ARM
	{
		HAL_UART_AbortReceive(&huart1);
		HAL_UART_Receive_DMA(&huart1, (uint8_t*)UART1_RX_BUFFER, sizeof(UART1_RX_BUFFER));
	}

	if(huart == &huart2)
	{
		HAL_UART_AbortReceive(&huart2);
		HAL_UART_Receive_DMA(&huart2, (uint8_t*)UART2_RX_BUFFER, sizeof(UART2_RX_BUFFER));
	}

	if(huart == &huart3)
	{
		HAL_UART_AbortReceive(&huart3);
		HAL_UART_Receive_DMA(&huart3, (uint8_t*)UART3_RX_BUFFER, sizeof(UART3_RX_BUFFER));
	}

	if(huart == &huart4)
	{
		HAL_UART_AbortReceive(&huart4);
		HAL_UART_Receive_DMA(&huart4, (uint8_t*)UART4_RX_BUFFER, sizeof(UART4_RX_BUFFER));
	}

	if(huart == &huart5)
	{
		HAL_UART_AbortReceive(&huart5);
		HAL_UART_Receive_DMA(&huart5, (uint8_t*)UART5_RX_BUFFER, sizeof(UART5_RX_BUFFER));
	}

	if(huart == &huart6)
	{
		HAL_UART_AbortReceive(&huart6);
		HAL_UART_Receive_DMA(&huart6, (uint8_t*)UART6_RX_BUFFER, sizeof(UART6_RX_BUFFER));

	}

}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_CAN1_Init();
  MX_I2C1_Init();
  MX_I2C3_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  MX_TIM8_Init();
  MX_TIM9_Init();
  MX_TIM10_Init();
  MX_TIM11_Init();
  MX_TIM12_Init();
  MX_TIM13_Init();
  MX_UART4_Init();
  MX_UART5_Init();
  MX_TIM6_Init();
  MX_USART2_UART_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  MX_USART6_UART_Init();
  MX_LWIP_Init();
  /* USER CODE BEGIN 2 */
  Robot_Init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if(udp_cnt >= 1)
	  {
		  MX_LWIP_Process();
		  udpClient_send();
		  udp_cnt = 0;
	  }

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */


/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
