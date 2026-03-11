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
#include "dma.h"
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
#include "MotionProfile.h"
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
#define MAX_ROTATION_PULSE 835
#define MAX_HORIZONTAL_PULSE 1955
#define MAX_VERTICAL_PULSE 2200
#define SEARCH_HORIZONTAL_PULSE 800

#define ENC_X_TO_CM 0.024691
#define ENC_Y_TO_CM 0.024907

#define WALL_THRESHOLD 20
#define WALL_DISTANCE_SETPOINT 10

uint8_t udp_cnt = 0;
uint8_t rst_state = 0;
int16_t rst_cnt = 0;

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
PID_t PID_x_enc;
PID_t PID_y_enc;
PID_t PID_w_mpu;
PID_t PID_x_ultra;
PID_t PID_y_cam;

PID_t PID_rot_enc;
PID_t PID_hor_enc;
PID_t PID_ver_enc;
PID_t PID_rot_cam;
PID_t PID_hor_cam;
PID_t PID_rot_box;

float kp_base = 45;
float ki_base = 2.5;
float kd_base = 0;

int16_t box_setpoint = 0;
int16_t box_feedback = 0;
int16_t vx = 0;
int16_t vy = 0;
int16_t vw = 0;
float global_x = 0;
float global_y = 0;
float yaw_degree = 0;
float yaw_flip   = 0;
float yaw_adjust = 0;
float yaw_radian = 0;
float base_pos_x = 0;
float base_pos_y = 0;
float base_pos_w = 0;
float set_x = 0;
float set_y = 0;
float set_w = 0;
uint8_t ultra_lost[3] = {0};
float ultra_avg_distance = 0;

MotionProfile x_profile;
MotionProfile y_profile;
MotionProfile w_profile;
MotionProfile rot_profile;
MotionProfile hor_profile;
MotionProfile ver_profile;

char UART1_RX_BUFFER[53]; //--- VGT ARM
char UART2_RX_BUFFER[9];
char UART3_RX_BUFFER[43];
char UART4_RX_BUFFER[53];
char UART5_RX_BUFFER[7];  //--- odom enc
char UART6_RX_BUFFER[7];  //--- NANO YAW

char UART1_TX_BUFFER[53] = "ABC";
char UART2_TX_BUFFER[9] = "ABC"; //--- RGB LED

//---- ARM
uint8_t arm_ready = 0;
uint8_t robot_start = 0;
uint8_t relay_state = 0;
int16_t set_speed_rot = 0;
int16_t set_speed_hor = 0;
int16_t set_speed_ver = 0;
int16_t set_position_rot = 0;
int16_t set_position_hor = 0;
int16_t set_position_ver = 0;
int16_t total_rot_enc = 0;
int16_t total_hor_enc = 0;
int16_t total_ver_enc = 0;
uint8_t arm_scanning_mode = 0;
uint8_t manual_arm = 0;
float 	set_rot = 0;
float 	set_hor = 0;
float 	set_ver = 0;
uint8_t cam_detected = 0;

//--- LED
typedef struct
{
	uint8_t state;
	uint8_t R, G, B;
	uint8_t brightness;
	uint8_t freq_ms;
} LED_t ;

LED_t RGB_LED;
uint8_t rgb_state = 0;

uint8_t arm_arm_state = 0;

void Robot_Init()
{
	Motor_Init(	&motorA,
				GPIOC, GPIO_PIN_15,
				GPIOC, GPIO_PIN_13,
				&htim11, TIM_CHANNEL_1, REVERSE_TRUE);

	Motor_Init(	&motorB,
				GPIOE, GPIO_PIN_3,
				GPIOE, GPIO_PIN_0,
				&htim10, TIM_CHANNEL_1, REVERSE_FALSE);

	Motor_Init(	&motorC,
				GPIOD, GPIO_PIN_10,
				GPIOD, GPIO_PIN_8,
				&htim12, TIM_CHANNEL_1, REVERSE_TRUE);

	Encoder_Init(&encA, &htim3, REVERSE_FALSE);
	Encoder_Init(&encB, &htim4, REVERSE_TRUE);
	Encoder_Init(&encC, &htim2, REVERSE_FALSE);

	PID_Init(&PID_rot_enc,  0.1, 0, 0,  3,  3, 25);
	PID_Init(&PID_hor_enc,  0.1, 0, 0, 10, 10, 25);
	PID_Init(&PID_ver_enc,  0.1, 0, 0, 60, 60, 50);
	PID_Init(&PID_rot_cam, 0.04, 0, 0,  1,  1, 25);
	PID_Init(&PID_hor_cam, 0.05, 0, 0,  5,  5, 25);
	PID_Init(&PID_rot_box, 0.02, 0, 0,  1,  1, 50);

    PID_Init(&PID_A, kp_base, ki_base, kd_base, 999, 999, 0);
    PID_Init(&PID_B, kp_base, ki_base, kd_base, 999, 999, 0);
    PID_Init(&PID_C, kp_base, ki_base, kd_base, 999, 999, 0);
    PID_Init(&PID_x_enc, 0.17, 0, 0, 20, 20, 5);
    PID_Init(&PID_y_enc, 0.17, 0, 0, 20, 20, 5);
    PID_Init(&PID_w_mpu, 0.45, 0, 8, 5, 5, 1);
    PID_Init(&PID_x_ultra, 0.5, 0, 0, 5, 5, 5);
    PID_Init(&PID_y_cam, 0.1, 0, 0, 5, 5, 1);

    MotionProfile_init(&x_profile, 65.0f, 50.0f, 50.0f, 5.0f);
    MotionProfile_init(&y_profile, 65.0f, 50.0f, 50.0f, 5.0f);
    MotionProfile_init(&w_profile, 40.0f, 5.0f, 5.0f, 5.0f);

    MotionProfile_init(&w_profile, 100.0f, 85.0f, 85.0f, 1.0f);
    MotionProfile_init(&rot_profile, 1000, 900, 900, 5);
    MotionProfile_init(&hor_profile, 3500, 3400, 3400, 5);
    MotionProfile_init(&ver_profile, 3500, 3400, 3400, 5);

    HAL_UART_Receive_DMA(&huart1, (uint8_t*)UART1_RX_BUFFER, sizeof(UART1_RX_BUFFER));
//    HAL_UART_Receive_DMA(&huart2, (uint8_t*)UART2_RX_BUFFER, sizeof(UART2_RX_BUFFER));
    HAL_UART_Receive_DMA(&huart3, (uint8_t*)UART3_RX_BUFFER, sizeof(UART3_RX_BUFFER));
    HAL_UART_Receive_DMA(&huart4, (uint8_t*)UART4_RX_BUFFER, sizeof(UART4_RX_BUFFER));
    HAL_UART_Receive_DMA(&huart5, (uint8_t*)UART5_RX_BUFFER, sizeof(UART5_RX_BUFFER));
    HAL_UART_Receive_DMA(&huart6, (uint8_t*)UART6_RX_BUFFER, sizeof(UART6_RX_BUFFER));

	udpClient_connect();

	HAL_TIM_Base_Start_IT(&htim6);
}


void Set_LED(LED_t *led, uint8_t state, uint8_t R, uint8_t G, uint8_t B, uint8_t brightness, uint8_t freq_ms)
{
	led -> state = state;
	led -> R = R;
	led -> G = G;
	led -> B = B;
	led -> brightness = brightness;
	led -> freq_ms = freq_ms;
}

void RGB_LED_Stop()
{
	static uint8_t state = 0;
	static uint16_t cnt_ms = 0;

	switch(state)
	{
		case 0:
			Set_LED(&RGB_LED, 1, 255, 0, 0, 10, 1);
			state++;
			break;
		case 1:
			if(cnt_ms >= 100)
			{
				state++;
				cnt_ms = 0;
			}
			else
			{
				cnt_ms++;
			}
			break;
		case 2:
			Set_LED(&RGB_LED, 1, 255, 0, 0, 0, 1);
			state++;
			break;
		case 3:
			if(cnt_ms >= 100)
			{
				state++;
				cnt_ms = 0;
			}
			else
			{
				cnt_ms++;
			}
			break;
		case 4:
			Set_LED(&RGB_LED, 1, 255, 0, 0, 10, 1);
			state++;
			break;
		case 5:
			if(cnt_ms >= 100)
			{
				state++;
				cnt_ms = 0;
			}
			else
			{
				cnt_ms++;
			}
			break;
		case 6:
			Set_LED(&RGB_LED, 1, 255, 0, 0, 0, 1);
			state++;
			break;
		case 7:
			if(cnt_ms >= 100)
			{
				state++;
				cnt_ms = 0;
			}
			else
			{
				cnt_ms++;
			}
			break;
		case 8:
			Set_LED(&RGB_LED, 1, 255, 0, 0, 10, 1);
			state++;
			break;
		case 9:
			if(cnt_ms >= 4000)
			{
				state = 0;
				cnt_ms = 0;
			}
			else
			{
				cnt_ms++;
			}
			break;
	}
}


void RGB_LED_Process(uint8_t state)
{
	switch(state)
	{
		case 0: // --- stop
			RGB_LED_Stop();
			break;
		case 1: //--- go
			Set_LED(&RGB_LED, 1, 0, 255, 0, 10, 1);
			break;
		case 2: //--- search
			Set_LED(&RGB_LED, 2, 0, 0, 0, 10, 1);
			break;
		case 3: //--- Daun
			Set_LED(&RGB_LED, 1, 199, 252, 0, 10, 1);
			break;
		case 4: //--- Ferro
			Set_LED(&RGB_LED, 1, 254, 0, 86, 10, 1);
			break;
		case 5: //--- Kertas
			Set_LED(&RGB_LED, 1, 0, 255, 206, 10, 1);
			break;
		case 6: //--- Non-Ferro
			Set_LED(&RGB_LED, 1, 255, 128, 0, 10, 1);
			break;
		case 7: //--- Plastik
			Set_LED(&RGB_LED, 1, 134, 34, 255, 10, 1);
			break;
		case 8: //--- Manual
			Set_LED(&RGB_LED, 1, 0, 0, 255, 10, 1);
			break;
	}

	memcpy(UART2_TX_BUFFER + 3, &RGB_LED, sizeof(RGB_LED));

	HAL_UART_Transmit_DMA(&huart2, (uint8_t*)UART2_TX_BUFFER, sizeof(UART2_TX_BUFFER));
}

float flip_yaw(float original_yaw)
{
   return 360.0f - fmodf(original_yaw, 360.0f);
}

void Yaw_Process()
{
	static uint8_t offset_once = 0;

	yaw_flip = flip_yaw(udp_tx.yaw_degree);

	if(yaw_flip > 180.001)
	{
		yaw_flip -= 360.001;
	}
	else if(yaw_flip < -180.001)
	{
        yaw_flip += 360.001;
	}

	if(!offset_once)
	{
		yaw_adjust = yaw_flip;
		offset_once++;
	}

	yaw_degree = yaw_flip - yaw_adjust;

	yaw_radian = yaw_degree * M_PI/180.0;
}

void Odometry_Process()
{
	static uint8_t odom_cnt_ms = 0;

	if(odom_cnt_ms >= 9)
	{

		global_x += (udp_tx.enc_x * cos(yaw_radian) - udp_tx.enc_y * sin(yaw_radian)) * ENC_X_TO_CM;
		global_y += (udp_tx.enc_x * sin(yaw_radian) + udp_tx.enc_y * cos(yaw_radian)) * ENC_Y_TO_CM;
		odom_cnt_ms = 0;
	}
	else
	{
		odom_cnt_ms++;
	}
}

void Main_Arm_Algorithm()
{
	if(!robot_start)
	{
		return;
	}

	static uint8_t arm_state = 0;
	static int16_t arm_cnt_ms = 0;
	static uint8_t trash_type = 0;

	arm_arm_state = arm_state;

	switch(arm_state)
	{
		case 0: //--- Arm up

			rgb_state = 1;
			manual_arm = 0;
			arm_scanning_mode = 0; //-- enc mode
			relay_state = 0;

			MotionProfile_set_target(&ver_profile, MAX_VERTICAL_PULSE);
			MotionProfile_update(&ver_profile, 0.001);
			set_ver = MotionProfile_get_position(&ver_profile);
			set_position_ver = (int16_t)set_ver;
			if(MotionProfile_finished(&ver_profile))
			{
				arm_state = 11;
			}

//			set_position_ver = MAX_VERTICAL_PULSE - 100;
//			arm_state = 111;
			break;

//		case 111:
//            if(arm_cnt_ms >= 2400)
//            {
//            	arm_state = 11;
//                arm_cnt_ms = 0;
//            }
//            else
//            {
//                arm_cnt_ms++;
//            }
//            break;

		case 1: //--- Arm hold rotate (at 0 position)

			rgb_state = 1;
			MotionProfile_set_target(&rot_profile, 0);
			MotionProfile_update(&rot_profile, 0.001);
			set_rot = MotionProfile_get_position(&rot_profile);
			set_position_rot = (int16_t)set_rot;
			if(MotionProfile_finished(&rot_profile))
			{
				arm_state++;
			}
			break;

		case 2: //--- Arm search position

			rgb_state = 1;
			MotionProfile_set_target(&hor_profile, SEARCH_HORIZONTAL_PULSE);
			MotionProfile_update(&hor_profile, 0.001);
			set_hor = MotionProfile_get_position(&hor_profile);
			set_position_hor = (int16_t)set_hor;
			if(MotionProfile_finished(&hor_profile))
			{
				arm_state++;
			}
			break;

		case 3: //--- wait for input (auto or manual)

			if(udp_tx.option) //--- manual mode
			{
				manual_arm = 1;
				arm_state = 99;
			}

//			if(udp_tx.down) //--- auto mode
//			if(udp_tx.buttons[1] == 0)
			if(udp_rx.trashDetected && abs(udp_rx.closestTrashX) <= 200 && !(ultra_lost[0] || ultra_lost[1] || ultra_lost[2]))
			{
				PID_Reset(&PID_rot_enc);
				PID_Reset(&PID_hor_enc);
				arm_scanning_mode = 1; //--- cam mode
				arm_state++;
				cam_detected = 1;
			}
			break;

		case 4: //--- Search Trash Auto
			rgb_state = 2;

			if(udp_tx.up)
			{
				PID_Reset(&PID_rot_cam);
				PID_Reset(&PID_hor_cam);
				arm_state = 0;
			}

			if(udp_rx.trashDetected)
			{
				cam_detected = 1;
				set_position_rot = 35;
				set_position_hor = 30;


                if(abs(PID_rot_cam.error) <= PID_rot_cam.tolerance && abs(PID_hor_cam.error) <= PID_hor_cam.tolerance)
                {
                    if(arm_cnt_ms >= 249)
                    {
                    	trash_type = udp_rx.trashType;
        				PID_Reset(&PID_rot_cam);
        				PID_Reset(&PID_hor_cam);
                        arm_scanning_mode = 0; //--- enc mode
                        arm_state++;
                        arm_cnt_ms = 0;
                    }
                    else
                    {
                        arm_cnt_ms++;
                    }
                }

			}
			else //--- stops arm movement if trash not detected
			{
				set_position_rot = PID_rot_cam.feedback;
				set_position_hor = PID_hor_cam.feedback;
				cam_detected = 0;
			}
			break;

		case 5: //--- go down while keeping rotation and horizontal position
			rgb_state = trash_type + 2; //--- offset of 2
			cam_detected = 1; //--- make sure to stop

			if(udp_tx.up)
			{
				PID_Reset(&PID_rot_cam);
				PID_Reset(&PID_hor_cam);
				arm_state = 0;
			}
			set_position_rot = PID_rot_enc.feedback;
			set_position_hor = PID_hor_enc.feedback;

			if(PID_ver_enc.feedback < 250)
			{
				relay_state = 1;
			}

			MotionProfile_set_target(&ver_profile, 0);
			MotionProfile_update(&ver_profile, 0.001);
			set_ver = MotionProfile_get_position(&ver_profile);
			set_position_ver = (int16_t)set_ver;

			if(MotionProfile_finished(&ver_profile))
			{
				arm_state++;
			}
			break;

		case 6: //--- go up, pick up trash

			rgb_state = trash_type + 2; //--- offset of 2

			MotionProfile_set_target(&ver_profile, MAX_VERTICAL_PULSE);
			MotionProfile_update(&ver_profile, 0.001);
			set_ver = MotionProfile_get_position(&ver_profile);
			set_position_ver = (int16_t)set_ver;
			if(MotionProfile_finished(&ver_profile))
			{
				arm_state++;
			}
			break;

		case 7: //--- extend MAX forward, ready for trash bin

			rgb_state = trash_type + 2; //--- offset of 2

			MotionProfile_set_target(&rot_profile, 0);
			MotionProfile_update(&rot_profile, 0.001);
			set_rot = MotionProfile_get_position(&rot_profile);
			set_position_rot = (int16_t)set_rot;

			if(MotionProfile_finished(&rot_profile))
			{
				arm_state++;
			}
			break;

		case 8:
			cam_detected = 0; //--- move again

			rgb_state = trash_type + 2; //--- offset of 2

			MotionProfile_set_target(&hor_profile, MAX_HORIZONTAL_PULSE);
			MotionProfile_update(&hor_profile, 0.001);
			set_hor = MotionProfile_get_position(&hor_profile);
			set_position_hor = (int16_t)set_hor;

			if(MotionProfile_finished(&hor_profile))
			{
				arm_state++;
			}
			break;

		case 9: //--- wait for input to go down, or go back to beginning state

			rgb_state = trash_type + 2; //--- offset of 2

//			if(udp_tx.down)
//			if(udp_tx.buttons[1] == 0)
			if(udp_rx.boxDetected && abs(udp_rx.cameraX - udp_rx.closestBoxX) <= 55)
			{
				cam_detected = 1;
				arm_scanning_mode = 2; //--- box mode
				PID_Reset(&PID_rot_enc);
				arm_state++;
			}

			if(udp_tx.up)
			{
				arm_state = 0;
			}
			break;

		case 10:
			rgb_state = trash_type + 2; //--- offset of 2

			if(udp_rx.boxDetected)
			{
				cam_detected = 1;
				set_position_rot = 45;

	            if(abs(PID_rot_box.error) <= PID_rot_box.tolerance)
	            {
	                if(arm_cnt_ms >= 249)
	                {
	    				PID_Reset(&PID_rot_box);
	                    arm_scanning_mode = 0;
	                    arm_state++;
	                    arm_cnt_ms = 0;
	                }
	                else
	                {
	                    arm_cnt_ms++;
	                }
	            }
			}
			else
			{
				cam_detected = 0;
				set_position_rot = PID_rot_box.feedback;
			}

			break;

		case 11: //--- go down and let go of trash


			set_position_rot = PID_rot_enc.feedback;

			rgb_state = trash_type + 2; //--- offset of 2

			relay_state = 0;

			MotionProfile_set_target(&ver_profile, 0);
			MotionProfile_update(&ver_profile, 0.001);
			set_ver = MotionProfile_get_position(&ver_profile);
			set_position_ver = (int16_t)set_ver;

			if(MotionProfile_finished(&ver_profile))
			{
                if(arm_cnt_ms >= 249) //2499
                {
                	cam_detected = 0;
                    arm_state = 0;
                    arm_cnt_ms = 0;
                }
                else
                {
                    arm_cnt_ms++;
                }
			}

//			set_position_ver = 200;
//			if(arm_cnt_ms >= 2499) //2499
//			{
////				cam_detected = 0;
//				arm_state = 0;
//				arm_cnt_ms = 0;
//			}
//			else
//			{
//				arm_cnt_ms++;
//			}

			break;

		case 21: //--- ignore this, this is just a test, nigga
			MotionProfile_set_target(&ver_profile, 0);
			MotionProfile_update(&ver_profile, 0.001);
			set_ver = MotionProfile_get_position(&ver_profile);
			set_position_ver = (int16_t)set_ver;
			if(MotionProfile_finished(&ver_profile))
			{
				arm_state = 0;
			}
			break;

		case 99: //--- MANUAL MODE, MOTHERFUCKER
		{
			rgb_state = 8;

            static float rot_scaled = 0;
            static float hor_scaled = 0;
            static uint8_t grab_state = 0;
            static uint8_t SUCK_state = 0;
            static uint8_t SUCK_TIME = 0;


            //---- control the GAWK GAWK 3000 SUCK PUMP
            switch (SUCK_state)
			{
				case 0:
					if(udp_tx.r1)
					{
						SUCK_state++;
					}
					break;
				case 1:
					relay_state = 1;

					if(SUCK_TIME >= 250)
					{
						SUCK_state++;
						SUCK_TIME = 0;
					}
					else
					{
						SUCK_TIME++;
					}
					break;
				case 2:
					if(udp_tx.r1)
					{
						SUCK_state++;
					}
					break;
				case 3:
					relay_state = 0;
					if(SUCK_TIME >= 250)
					{
						SUCK_state = 0;
						SUCK_TIME = 0;
					}
					else
					{
						SUCK_TIME++;
					}
					break;
			}

            //--- Control the forward and rotation movement of arm
            if(udp_tx.left && rot_scaled < 300)
            {
                rot_scaled += 0.2;
            }
            else if(udp_tx.right && rot_scaled > -300)
            {
                rot_scaled -= 0.2;
            }

            if(udp_tx.up && hor_scaled < MAX_HORIZONTAL_PULSE)
            {
                hor_scaled += 1;
            }
            else if(udp_tx.down && hor_scaled > 10)
            {
                hor_scaled -= 1;
            }

            set_position_rot = rot_scaled;
            set_position_hor = hor_scaled;

            //-- Control UP and DOWN movement of your MUM
            switch(grab_state)
            {
				case 0:
					if(udp_tx.l1)
					{
						grab_state++;
					}
					break;

				case 1:
					MotionProfile_set_target(&ver_profile, 0);
					MotionProfile_update(&ver_profile, 0.001);
					set_ver = MotionProfile_get_position(&ver_profile);
					set_position_ver = (int16_t)set_ver;

					if(MotionProfile_finished(&ver_profile))
					{
		                if(arm_cnt_ms >= 249)
		                {
		                    grab_state++;
		                    arm_cnt_ms = 0;
		                }
		                else
		                {
		                    arm_cnt_ms++;
		                }
					}
					break;

				case 2:
					if(udp_tx.l1)
					{
						grab_state++;
					}
					break;

				case 3:
					MotionProfile_set_target(&ver_profile, MAX_VERTICAL_PULSE);
					MotionProfile_update(&ver_profile, 0.001);
					set_ver = MotionProfile_get_position(&ver_profile);
					set_position_ver = (int16_t)set_ver;

					if(MotionProfile_finished(&ver_profile))
					{
		                if(arm_cnt_ms >= 249)
		                {
		                    grab_state = 0;
		                    arm_cnt_ms = 0;
		                }
		                else
		                {
		                    arm_cnt_ms++;
		                }
					}
					break;
            }


            //--- GO BACK TO AUTO MODE
            if(udp_tx.share)
            {
                manual_arm = 0;
                rot_scaled = 0;
                hor_scaled = 0;
                grab_state = 0;
                SUCK_state = 0;

                PID_Reset(&PID_ver_enc);
                PID_Reset(&PID_rot_enc);
                PID_Reset(&PID_hor_enc);

                arm_state = 0;
            }

			break;
		}

	}

}

float get_weighted_distance()
{
    float weighted_sum = 0;
    float total_weight = 0;

    if(!ultra_lost[0]) // RIGHT
    {
        weighted_sum += udp_tx.ultrasonic[0] * 1.0f;
        total_weight += 1.0f;
    }

    if(!ultra_lost[1]) // CENTER
    {
        weighted_sum += udp_tx.ultrasonic[1] * 2.0f;
        total_weight += 2.0f;
    }

    if(!ultra_lost[2]) // LEFT
    {
        weighted_sum += udp_tx.ultrasonic[2] * 1.0f;
        total_weight += 1.0f;
    }

    if(total_weight > 0)
    {
        return weighted_sum / total_weight;
    }

    return WALL_DISTANCE_SETPOINT; // Fallback
}

void Main_Base_Algorithm()
{
	if(!robot_start)
	{
		yaw_adjust = yaw_flip;
		global_x = 0;
		global_y = 0;
		ultra_avg_distance = 0;
		Encoder_ResetCount(&encA);
		Encoder_ResetCount(&encB);
		Encoder_ResetCount(&encC);
		box_feedback = 0;
		return;
	}

	vx = vy = vw = 0;

//	static uint8_t base_state = 0;
//	static uint16_t base_cnt_ms = 0;
//
//	vy = -Controller_Drift(udp_tx.lX, 12);
//	vx = Controller_Drift(udp_tx.lY, 12);
//	int16_t rx = -Controller_Drift(udp_tx.rX, 12);
//
//	vx = map(vx, -128, 127, -10, 10);
//	vy = map(vy, -128, 127, -10, 10);
//	rx = map(rx, -128, 127, -3, 3);
//
//	vw = rx;

//	set_w += (float) rx * 0.3; //--- ini scale nya
//
//    set_w = fmodf(set_w, 360.0f);
//    if (set_w > 180.0f)
//    {
//    	set_w -= 360.0f;
//    }
//    else if (set_w < -180.0f)
//    {
//    	set_w += 360.0f;
//    }






//---- AUTO

//	ultra_lost[0] = ( udp_tx.ultrasonic[0] > WALL_THRESHOLD ); //-- RIGHT
//	ultra_lost[1] = ( udp_tx.ultrasonic[1] > WALL_THRESHOLD ); //-- CENTER
//	ultra_lost[2] = ( udp_tx.ultrasonic[2] > WALL_THRESHOLD ); //-- LEFT
//	ultra_avg_distance = get_weighted_distance();
//
//	if(cam_detected)
//	{
//		set_x = WALL_DISTANCE_SETPOINT;
//		set_y = 0;
//		set_w = 0;
//
//		vy = 0; //--- STOP
//
////		if(base_state == 0)
////		{
////			base_state = 1;
////		}
////		else
////		{
////			base_state = 0;
////		}
//	}
//
//	else
//	{
//		switch(base_state)
//		{
//			case 0:
//				set_x = WALL_DISTANCE_SETPOINT;
//				set_y = 0;
//				set_w = 0;
//
//				vy = -4; //--- go right
//
//				if(ultra_lost[0])
//				{
//					base_state++;
//				}
//				break;
//
//			case 1:
//				set_x = WALL_DISTANCE_SETPOINT;
//				set_y = 0;
//				set_w = 3;
//
//				vy = 4; //--- go left
//
//				if(ultra_lost[2])
//				{
//					base_state = 0;
//				}
//				break;
//		}
//	}


//	MotionProfile_set_target(&w_profile, set_w);
//	MotionProfile_update(&w_profile, 0.001);
//	base_pos_w = MotionProfile_get_position(&w_profile);
//
//	MotionProfile_set_target(&y_profile, set_y);
//	MotionProfile_update(&y_profile, 0.001);
//	base_pos_y = MotionProfile_get_position(&y_profile);
//
//	MotionProfile_set_target(&x_profile, set_x);
//	MotionProfile_update(&x_profile, 0.001);
//	base_pos_x = MotionProfile_get_position(&x_profile);
}

void PID_Arm_Position()
{
	static uint16_t timer = 0;

	if(timer >= 9)
	{
		total_rot_enc += udp_tx.enc_1;
        total_hor_enc += udp_tx.enc_2;
        total_ver_enc += udp_tx.enc_3;


        switch(arm_scanning_mode)
        {
			case 0: //--- enc mode full

				PID_Update(&PID_rot_enc, set_position_rot, total_rot_enc);

				set_speed_rot = (int16_t)PID_rot_enc.output;

				if(abs(PID_rot_enc.error) < PID_rot_enc.tolerance)
				{
					set_speed_rot = 0;
				}
		        if(total_rot_enc < -300 && PID_rot_enc.output <= 0)
		        {
		        	set_speed_rot = 0;
		        }
		        if(total_rot_enc > 300 && PID_rot_enc.output >= 0)
		        {
		        	set_speed_rot = 0;
		        }


				PID_Update(&PID_hor_enc, set_position_hor, total_hor_enc);

				set_speed_hor = (int16_t)PID_hor_enc.output;

				if(abs(PID_hor_enc.error) < PID_hor_enc.tolerance)
				{
					set_speed_hor = 0;
				}
		        if(total_hor_enc < 10 && PID_hor_enc.output <= 0)
		        {
		        	set_speed_hor = 0;
		        }
		        if(total_hor_enc > MAX_HORIZONTAL_PULSE && PID_hor_enc.output >= 0)
		        {
		        	set_speed_hor = 0;
		        }
				break;

			case 1: //--- cam mode full

				PID_Update(&PID_rot_cam, set_position_rot, udp_rx.closestTrashX);

				set_speed_rot = (int16_t)PID_rot_cam.output;

				if(abs(PID_rot_cam.error) < PID_rot_cam.tolerance)
				{
					set_speed_rot = 0;
				}
		        if(total_rot_enc < -300 && PID_rot_cam.output <= 0)
		        {
		        	set_speed_rot = 0;
		        }
		        if(total_rot_enc > 300 && PID_rot_cam.output >= 0)
		        {
		        	set_speed_rot = 0;
		        }


				PID_Update(&PID_hor_cam, set_position_hor, udp_rx.closestTrashY);

				set_speed_hor = (int16_t)PID_hor_cam.output;

				if(abs(PID_hor_cam.error) < PID_hor_cam.tolerance)
				{
					set_speed_hor = 0;
				}
		        if(total_hor_enc < 200 && PID_hor_cam.output <= 0)
		        {
		        	set_speed_hor = 0;
		        }
		        if(total_hor_enc > 1400 && PID_hor_cam.output >= 0)
		        {
		        	set_speed_hor = 0;
		        }
				break;

			case 2: //--- box mode

				PID_Update(&PID_rot_box, set_position_rot, (udp_rx.cameraX - udp_rx.closestBoxX));

				set_speed_rot = (int16_t)PID_rot_box.output;

				if(abs(PID_rot_box.error) < PID_rot_box.tolerance)
				{
					set_speed_rot = 0;
				}
		        if(total_rot_enc < -300 && PID_rot_box.output <= 0)
		        {
		        	set_speed_rot = 0;
		        }
		        if(total_rot_enc > 300 && PID_rot_box.output >= 0)
		        {
		        	set_speed_rot = 0;
		        }


				PID_Update(&PID_hor_enc, set_position_hor, total_hor_enc);

				set_speed_hor = (int16_t)PID_hor_enc.output;

				if(abs(PID_hor_enc.error) < PID_hor_enc.tolerance)
				{
					set_speed_hor = 0;
				}
		        if(total_hor_enc < 10 && PID_hor_enc.output <= 0)
		        {
		        	set_speed_hor = 0;
		        }
		        if(total_hor_enc > MAX_HORIZONTAL_PULSE && PID_hor_enc.output >= 0)
		        {
		        	set_speed_hor = 0;
		        }
				break;
        }

//		if(arm_scanning_mode)
//		{
//			PID_Update(&PID_rot_cam, set_position_rot, udp_rx.closestTrashX);
//
//			set_speed_rot = (int16_t)PID_rot_cam.output;
//
//			if(abs(PID_rot_cam.error) < PID_rot_cam.tolerance)
//			{
//				set_speed_rot = 0;
//			}
//	        if(total_rot_enc < -300 && PID_rot_cam.output <= 0)
//	        {
//	        	set_speed_rot = 0;
//	        }
//	        if(total_rot_enc > 300 && PID_rot_cam.output >= 0)
//	        {
//	        	set_speed_rot = 0;
//	        }
//
//
//			PID_Update(&PID_hor_cam, set_position_hor, udp_rx.closestTrashY);
//
//			set_speed_hor = (int16_t)PID_hor_cam.output;
//
//			if(abs(PID_hor_cam.error) < PID_hor_cam.tolerance)
//			{
//				set_speed_hor = 0;
//			}
//	        if(total_hor_enc < 200 && PID_hor_cam.output <= 0)
//	        {
//	        	set_speed_hor = 0;
//	        }
//	        if(total_hor_enc > 1400 && PID_hor_cam.output >= 0)
//	        {
//	        	set_speed_hor = 0;
//	        }
//		}
//		else
//		{
//			PID_Update(&PID_rot_enc, set_position_rot, total_rot_enc);
//
//			set_speed_rot = (int16_t)PID_rot_enc.output;
//
//			if(abs(PID_rot_enc.error) < PID_rot_enc.tolerance)
//			{
//				set_speed_rot = 0;
//			}
//	        if(total_rot_enc < -300 && PID_rot_enc.output <= 0)
//	        {
//	        	set_speed_rot = 0;
//	        }
//	        if(total_rot_enc > 300 && PID_rot_enc.output >= 0)
//	        {
//	        	set_speed_rot = 0;
//	        }
//
//
//			PID_Update(&PID_hor_enc, set_position_hor, total_hor_enc);
//
//			set_speed_hor = (int16_t)PID_hor_enc.output;
//
//			if(abs(PID_hor_enc.error) < PID_hor_enc.tolerance)
//			{
//				set_speed_hor = 0;
//			}
//	        if(total_hor_enc < 10 && PID_hor_enc.output <= 0)
//	        {
//	        	set_speed_hor = 0;
//	        }
//	        if(total_hor_enc > MAX_HORIZONTAL_PULSE && PID_hor_enc.output >= 0)
//	        {
//	        	set_speed_hor = 0;
//	        }
//		}
//
//
//		if(arm_box_mode)
//		{
//			PID_Update(&PID_rot_box, set_position_rot, (udp_rx.cameraX - udp_rx.closestBoxX));
//
//			set_speed_rot = (int16_t)PID_rot_box.output;
//
//			if(abs(PID_rot_box.error) < PID_rot_box.tolerance)
//			{
//				set_speed_rot = 0;
//			}
//	        if(total_rot_enc < -300 && PID_rot_box.output <= 0)
//	        {
//	        	set_speed_rot = 0;
//	        }
//	        if(total_rot_enc > 300 && PID_rot_box.output >= 0)
//	        {
//	        	set_speed_rot = 0;
//	        }
//
//
//			PID_Update(&PID_hor_enc, set_position_hor, total_hor_enc);
//
//			set_speed_hor = (int16_t)PID_hor_enc.output;
//
//			if(abs(PID_hor_enc.error) < PID_hor_enc.tolerance)
//			{
//				set_speed_hor = 0;
//			}
//	        if(total_hor_enc < 10 && PID_hor_enc.output <= 0)
//	        {
//	        	set_speed_hor = 0;
//	        }
//	        if(total_hor_enc > MAX_HORIZONTAL_PULSE && PID_hor_enc.output >= 0)
//	        {
//	        	set_speed_hor = 0;
//	        }
//		}

		PID_Update(&PID_ver_enc, set_position_ver, total_ver_enc);

		set_speed_ver = (int16_t)PID_ver_enc.output;

		if(abs(PID_ver_enc.error) < PID_ver_enc.tolerance)
		{
			set_speed_ver = 0;
		}

		timer = 0;
	}
	else
	{
		timer++;
	}
}

void PID_Base_Position()
{
	static uint16_t timer = 0;

	if(timer >= 9)
	{

//		PID_Update(&PID_x_enc, base_pos_x, global_x);
//		vx = (int16_t)PID_x_enc.output;



//		PID_Update(&PID_y_enc, base_pos_y, global_y);
//		vy = (int16_t)PID_y_enc.output;

		//--- auto
//		PID_Update(&PID_x_ultra, set_x, ultra_avg_distance);
//		vx = -(int16_t)PID_x_ultra.output;
//		PID_Update_Rotate(&PID_w_mpu, set_w, yaw_degree);
//		vw = (int16_t)PID_w_mpu.output;

//		PID_Update(&PID_y_cam, box_setpoint, box_feedback);
//		vy = (int16_t)PID_y_cam.output;

		timer = 0;
	}
	else
	{
		timer++;
	}
}

void PID_Base_Speed()
{
	static uint16_t timer = 0;

	if(timer >= 9)
	{
		int16_t va = Kinematics_Triangle(MOTOR_A, vx, vy, vw);
		int16_t vb = Kinematics_Triangle(MOTOR_B, vx, vy, vw);
		int16_t vc = Kinematics_Triangle(MOTOR_C, vx, vy, vw);

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

		PID_Update(&PID_A, va, (float)encA.count);
		PID_Update(&PID_B, vb, (float)encB.count);
		PID_Update(&PID_C, vc, (float)encC.count);

		Motor_Run(&motorA, (int16_t)PID_A.output);
		Motor_Run(&motorB, (int16_t)PID_B.output);
		Motor_Run(&motorC, (int16_t)PID_C.output);

		timer = 0;
	}
	else
	{
		timer++;
	}
}

void LED_Blink()
{
	static uint16_t timer = 0;
	static uint8_t state = 0;

	if(timer >= 99)
	{
		state = !(state);
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_1, state);
		timer = 0;
	}
	else
	{
		timer++;
	}
}

void Arm_Transmit_UART()
{
	//--- VGT ARM
	memcpy(UART1_TX_BUFFER +  3, &robot_start, 1);
	memcpy(UART1_TX_BUFFER +  4, &rst_state, 1);
	memcpy(UART1_TX_BUFFER +  5, &relay_state, 1);
	memcpy(UART1_TX_BUFFER +  6, &set_speed_rot, 2);
	memcpy(UART1_TX_BUFFER +  8, &set_speed_hor, 2);
	memcpy(UART1_TX_BUFFER + 10, &set_speed_ver, 2);

	HAL_UART_Transmit_DMA(&huart1, (uint8_t*)UART1_TX_BUFFER, sizeof(UART1_TX_BUFFER));
}

void Read_Buttons()
{
	udp_tx.start_button = HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_0);
	udp_tx.reset_button = HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_1);
	udp_tx.buttons[0] = HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_4);
	udp_tx.buttons[1] = HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_7);
	udp_tx.buttons[2] = HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_3);

	if(udp_tx.start_button == 0 && arm_ready)
	{
		robot_start = 1;
		rst_state = 0;
	}

	if(udp_tx.reset_button == 0)
	{
		robot_start = 0;
		rst_state = 1;
	}
}

void Reset_Robot()
{
	if(rst_state)
	{
		if(rst_cnt >= 500)
		{
			NVIC_SystemReset();
		}
		else
		{
			rst_cnt++;
		}
	}
}

void Robot_Loop()
{
	//--- ROBOT
	if(!robot_start)
	{
		rgb_state = 0;
	}
	else
	{
		rgb_state = 1;
	}

	LED_Blink();
	Read_Buttons();
	Reset_Robot();
	//--- ARM
	Main_Arm_Algorithm();
	PID_Arm_Position();
	Arm_Transmit_UART();
	//--- BASE
	Main_Base_Algorithm();
	Yaw_Process();
	Odometry_Process();
	PID_Base_Position();
	PID_Base_Speed();

	RGB_LED_Process(rgb_state);

	udp_cnt++;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim == &htim6)
	{
		Robot_Loop();
	}
}


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart == &huart1) //--- VGT ARM
	{
		if(UART1_RX_BUFFER[0] == 'A' && UART1_RX_BUFFER[1] == 'B' && UART1_RX_BUFFER[2] == 'C')
		{
			/* Save UDP */
			memcpy(&udp_tx.buttons[3], UART1_RX_BUFFER + 3, 1);
			memcpy(&udp_tx.buttons[4], UART1_RX_BUFFER + 4, 1);
			memcpy(&udp_tx.lim2, UART1_RX_BUFFER + 5, 1);
			memcpy(&udp_tx.lim3, UART1_RX_BUFFER + 6, 1);
			memcpy(&udp_tx.enc_1, UART1_RX_BUFFER + 7, 2);
			memcpy(&udp_tx.enc_2, UART1_RX_BUFFER + 9, 2);
			memcpy(&udp_tx.enc_3, UART1_RX_BUFFER + 11, 2);
			memcpy(&arm_ready, UART1_RX_BUFFER + 13, 1);
		}
		HAL_UART_Receive_DMA(&huart1, (uint8_t*)UART1_RX_BUFFER, sizeof(UART1_RX_BUFFER));
	}

//	if(huart == &huart2)
//	{
//		HAL_UART_Receive_DMA(&huart2, (uint8_t*)UART2_RX_BUFFER, sizeof(UART2_RX_BUFFER));
//	}

	if(huart == &huart3)
	{
		if(UART3_RX_BUFFER[0] == 'A' && UART3_RX_BUFFER[1] == 'B' && UART3_RX_BUFFER[2] == 'C')
		{
			/* Save UDP */
			memcpy(&udp_tx.rX, UART3_RX_BUFFER + 3, 1);
			memcpy(&udp_tx.rY, UART3_RX_BUFFER + 4, 1);
			memcpy(&udp_tx.lX, UART3_RX_BUFFER + 5, 1);
			memcpy(&udp_tx.lY, UART3_RX_BUFFER + 6, 1);

			memcpy(&udp_tx.r2, UART3_RX_BUFFER + 7, 1);
			memcpy(&udp_tx.l2, UART3_RX_BUFFER + 8, 1);
			memcpy(&udp_tx.r1, UART3_RX_BUFFER + 9, 1);
			memcpy(&udp_tx.l1, UART3_RX_BUFFER + 10, 1);
			memcpy(&udp_tx.r3, UART3_RX_BUFFER + 11, 1);
			memcpy(&udp_tx.l3, UART3_RX_BUFFER + 12, 1);

			memcpy(&udp_tx.crs, UART3_RX_BUFFER + 13, 1);
			memcpy(&udp_tx.sqr, UART3_RX_BUFFER + 14, 1);
			memcpy(&udp_tx.tri, UART3_RX_BUFFER + 15, 1);
			memcpy(&udp_tx.cir, UART3_RX_BUFFER + 16, 1);
			memcpy(&udp_tx.up, UART3_RX_BUFFER + 17, 1);
			memcpy(&udp_tx.down, UART3_RX_BUFFER + 18, 1);
			memcpy(&udp_tx.right, UART3_RX_BUFFER + 19, 1);
			memcpy(&udp_tx.left, UART3_RX_BUFFER + 20, 1);
			memcpy(&udp_tx.share, UART3_RX_BUFFER + 21, 1);
			memcpy(&udp_tx.option, UART3_RX_BUFFER + 22, 1);

			memcpy(&udp_tx.ps, UART3_RX_BUFFER + 23, 1);
			memcpy(&udp_tx.touchpad, UART3_RX_BUFFER + 24, 1);
			memcpy(&udp_tx.battery, UART3_RX_BUFFER + 25, 1);

			memcpy(&udp_tx.gX, UART3_RX_BUFFER + 26, 2);
			memcpy(&udp_tx.gY, UART3_RX_BUFFER + 28, 2);
			memcpy(&udp_tx.gZ, UART3_RX_BUFFER + 30, 2);
			memcpy(&udp_tx.aX, UART3_RX_BUFFER + 32, 2);
			memcpy(&udp_tx.aY, UART3_RX_BUFFER + 34, 2);
			memcpy(&udp_tx.aZ, UART3_RX_BUFFER + 36, 2);
		}
		HAL_UART_Receive_DMA(&huart3, (uint8_t*)UART3_RX_BUFFER, sizeof(UART3_RX_BUFFER));
	}

	if(huart == &huart4)
	{
		if(UART4_RX_BUFFER[0] == 'A' && UART4_RX_BUFFER[1] == 'B' && UART4_RX_BUFFER[2] == 'C')
		{
			/* Save UDP */
			memcpy(&udp_tx.ultrasonic[0], UART4_RX_BUFFER + 3, 2);
			memcpy(&udp_tx.ultrasonic[1], UART4_RX_BUFFER + 5, 2);
			memcpy(&udp_tx.ultrasonic[2], UART4_RX_BUFFER + 7, 2);
			memcpy(&udp_tx.ultrasonic[3], UART4_RX_BUFFER + 9, 2);
		}
		HAL_UART_Receive_DMA(&huart4, (uint8_t*)UART4_RX_BUFFER, sizeof(UART4_RX_BUFFER));
	}

	if(huart == &huart5)
	{
		if(UART5_RX_BUFFER[0] == 'A' && UART5_RX_BUFFER[1] == 'B' && UART5_RX_BUFFER[2] == 'C')
		{
			/* Save UDP */
			memcpy(&udp_tx.enc_x, UART5_RX_BUFFER + 3, 2);
			memcpy(&udp_tx.enc_y, UART5_RX_BUFFER + 5, 2);
		}
		HAL_UART_Receive_DMA(&huart5, (uint8_t*)UART5_RX_BUFFER, sizeof(UART5_RX_BUFFER));
	}

	if(huart == &huart6) //--- NANO YAW
	{
		if(UART6_RX_BUFFER[0] == 'A' && UART6_RX_BUFFER[1] == 'B' && UART6_RX_BUFFER[2] == 'C')
		{
			/* Save UDP */
			memcpy(&udp_tx.yaw_degree, UART6_RX_BUFFER + 3, 4);
		}
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

	if(huart == &huart6) //--- NANO YAW
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

	if(huart == &huart6) //--- NANO YAW
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
//		  udpClient_send();
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
