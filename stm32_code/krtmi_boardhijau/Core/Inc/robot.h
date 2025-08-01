/*
 * robot.h
 *
 *  Created on: Jun 24, 2025
 *      Author: ALFA
 */

#ifndef INC_ROBOT_H_
#define INC_ROBOT_H_

#include "udpClientRAW.h"
#include "controller.h"
#include "motor.h"
#include "pid.h"

extern Controller_t input;

extern Motor_t motorA;
extern Motor_t motorB;
extern Motor_t motorC;

extern Encoder_t encA;
extern Encoder_t encB;
extern Encoder_t encC;
extern Encoder_t encX;
extern Encoder_t encY;

extern PID_t PID_A;
extern PID_t PID_B;
extern PID_t PID_C;
extern PID_t PID_VY;
extern PID_t PID_VW;

extern int16_t vx;
extern int16_t vy;
extern int16_t vw;
extern int16_t vx_controller;
extern int16_t vy_controller;
extern int16_t vw_controller;

extern float kp;
extern float ki;
extern float kd;

extern char UART2_RX[23];
extern char UART3_RX[43];
extern char UART4_RX[53];
extern char UART5_RX[23];

extern float yaw_degree;
extern float yaw_radian;
extern float yaw_adjust;
extern uint16_t UltraSonic[4];


void Robot_Init();
void Robot_Motor();
void Robot_LED_Blink();
void Robot_UDP();
void Robot_Loop();

#endif /* INC_ROBOT_H_ */
