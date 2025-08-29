/*
  ***************************************************************************************************************
  ***************************************************************************************************************
  ***************************************************************************************************************

  File:		  udpClientRAW.c
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

/* IMPLEMENTATION FOR UDP CLIENT :   source:https://www.geeksforgeeks.org/udp-server-client-implementation-c/

1. Create UDP socket.
2. Send message to server.
3. Wait until response from server is received.
4. Process reply and go back to step 2, if necessary.
5. Close socket descriptor and exit.
*/

#include "udpClientRAW.h"

typedef struct
{
	uint32_t a;
	uint32_t b;
	uint32_t c;
	uint32_t d;
	uint32_t port;
} ip_address_t ;

ip_address_t stm32_addr = { .a = 192, .b = 168, .c = 13, .d = 111, .port = 1555 };
ip_address_t pc_addr    = { .a = 192, .b = 168, .c = 13, .d = 100, .port = 1556 };

struct udp_pcb *upcb;
ip_addr_t stm32IPaddr;    //-- stm32
ip_addr_t pcIPaddr;  	  //-- NUC/MINI PC

udpTx_t udp_tx;
udpRx_t udp_rx;
char udp_rx_buffer[UDP_BUFFER_SIZE];
char udp_tx_buffer[UDP_BUFFER_SIZE] = "ABC";



void udp_receive_callback(void *arg, struct udp_pcb *upcb, struct pbuf *p, const ip_addr_t *addr, u16_t port);

void udpClient_connect(void)
{
	err_t err;
	/* 1. Create a new UDP control block  */
	upcb = udp_new();
	/* Bind the block to module's IP and port */
	IP_ADDR4(&stm32IPaddr, stm32_addr.a, stm32_addr.b, stm32_addr.c, stm32_addr.d); //--- stm32
	udp_bind(upcb, &stm32IPaddr, stm32_addr.port);
	/* configure destination IP address and port */
	IP_ADDR4(&pcIPaddr, pc_addr.a, pc_addr.b, pc_addr.c, pc_addr.d); //--- NUC/MINI PC
	err= udp_connect(upcb, &pcIPaddr, pc_addr.port);

	if (err == ERR_OK)
	{
		udp_recv(upcb, udp_receive_callback, NULL);
	}
}


void udpClient_send(void)
{
  struct pbuf *txBuf;

  int len = sizeof(udp_tx_buffer);

  memcpy(udp_tx_buffer +  3, &udp_tx, sizeof(udpTx_t));

  txBuf = pbuf_alloc(PBUF_TRANSPORT, len, PBUF_POOL);

  if (txBuf != NULL)
  {
    pbuf_take(txBuf, udp_tx_buffer, len);

    udp_sendto(upcb, txBuf, &pcIPaddr, pc_addr.port);

    pbuf_free(txBuf);
  }
}


void udp_receive_callback(void *arg, struct udp_pcb *upcb, struct pbuf *p, const ip_addr_t *addr, u16_t port)
{
	/* Copy the data from the pbuf */
	memcpy(udp_rx_buffer, p->payload, p->len);

	memcpy(&udp_rx, udp_rx_buffer +  3, sizeof(udpRx_t));

//	memcpy(&udp_rx.robot_start, udp_rx_buffer + 3, 1);
//	memcpy(&udp_rx.robot_reset, udp_rx_buffer + 4, 1);
//	memcpy(&udp_rx.motorA_setpoint, udp_rx_buffer + 5, 2);
//	memcpy(&udp_rx.motorB_setpoint, udp_rx_buffer + 7, 2);
//	memcpy(&udp_rx.motorC_setpoint, udp_rx_buffer + 9, 2);
//	memcpy(&udp_rx.rotation_setpoint, udp_rx_buffer + 11, 2);
//	memcpy(&udp_rx.horizontal_setpoint, udp_rx_buffer + 13, 2);
//	memcpy(&udp_rx.vertical_setpoint, udp_rx_buffer + 15, 2);
//	memcpy(&udp_rx.relay_state, udp_rx_buffer + 17, 1);

	for(int i = 0; i < 10; i++)
	{
		memcpy(&udp_rx.indicator[i], udp_rx_buffer + 18 + i, 1);
	}

	pbuf_free(p);
}


