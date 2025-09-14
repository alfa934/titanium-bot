/*
  ***************************************************************************************************************
  ***************************************************************************************************************
  ***************************************************************************************************************

  File:		  udpClientRAW.h
  Author:     ControllersTech.com
  Updated:    Jul 23, 2021

  ***************************************************************************************************************
  Copyright (C) 2017 ControllersTech.com

  This is a free software under the GNU license, you can redistribute it and/or modify it under the terms
  of the GNU General Public License version 3 as published by the Free Software Foundation.
  This software library is shared with public for educational purposes, without WARRANTY and Author is not liable for any damages caused directly
  or indirectly by this software, read more about this on the GNU General Public License.

  ***************************************************************************************************************
*/


#ifndef INC_UDPCLIENTRAW_H_
#define INC_UDPCLIENTRAW_H_

#include "main.h"

#include "lwip/pbuf.h"
#include "lwip/udp.h"
#include "lwip/tcp.h"

#include "stdio.h"
#include "string.h"


typedef struct
{
	int16_t enc_a;
	int16_t enc_b;
	int16_t enc_c;

	int16_t enc_x;
	int16_t enc_y;

	int16_t enc_1;
	int16_t enc_2;
	int16_t enc_3;

	float 	yaw_degree;

	uint16_t ultrasonic[4];

	uint8_t lim2;
	uint8_t lim3;

	uint8_t start_button;
	uint8_t reset_button;
	uint8_t buttons[5];

	int8_t rX;
	int8_t rY;
	int8_t lX;
	int8_t lY;
	uint8_t r2;
	uint8_t l2;
	uint8_t r1;
	uint8_t l1;
	uint8_t r3;
	uint8_t l3;
	uint8_t crs;
	uint8_t sqr;
	uint8_t tri;
	uint8_t cir;
	uint8_t up;
	uint8_t down;
	uint8_t right;
	uint8_t left;
	uint8_t share;
	uint8_t option;

	uint8_t ps;
	uint8_t touchpad;
	uint8_t battery;

	int16_t gX, gY, gZ;
	int16_t aX, aY, aZ;
} udpTx_t ;

typedef struct
{
	uint8_t robot_start;
	uint8_t robot_reset;

	int16_t motorA_setpoint;
	int16_t motorB_setpoint;
	int16_t motorC_setpoint;

	int16_t rotation_setpoint;
	int16_t horizontal_setpoint;
	int16_t vertical_setpoint;
	uint8_t relay_state;

	uint8_t indicator[10];

} udpRx_t ;

#define UDP_BUFFER_SIZE 81

extern udpTx_t udp_tx;
extern udpRx_t udp_rx;
extern char udp_rx_buffer[UDP_BUFFER_SIZE];
extern char udp_tx_buffer[UDP_BUFFER_SIZE];


void udpClient_connect(void);
void udpClient_send(void);

#endif /* INC_UDPCLIENTRAW_H_ */
