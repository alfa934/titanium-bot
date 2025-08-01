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

	float 	yaw_degree;

	uint16_t ultrasonic[4];

} udpTx_t ;

typedef struct
{
	int16_t motor_a;
	int16_t motor_b;
	int16_t motor_c;
} udpRx_t ;

extern udpTx_t udp_tx;
extern udpRx_t udp_rx;
extern char udp_rx_buffer[64];
extern char udp_tx_buffer[64];


void udpClient_connect(void);
void udpClient_send(void);

#endif /* INC_UDPCLIENTRAW_H_ */
