/**
  ******************************************************************************
  * @file    LwIP/LwIP_UDP_Echo_Server/Src/udp_echoserver.c
  * @author  MCD Application Team
  * @brief   UDP echo server
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2017 STMicroelectronics International N.V.
  * All rights reserved.</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice,
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other
  *    contributors to this software may be used to endorse or promote products
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under
  *    this license is void and will automatically terminate your rights under
  *    this license.
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "lwip/pbuf.h"
#include "lwip/udp.h"
#include "lwip/tcp.h"
#include <string.h>
#include <stdio.h>
#include "udp_echoserver.h"

#define UDP_ECHO_SERVER_DEBUG                 1

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define UDP_SERVER_PORT    5010     /* define the UDP local connection port */
#define UDP_CLIENT_PORT    50299    /* define the UDP remote connection port */

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
void udp_echoserver_receive_callback(void *arg, struct udp_pcb *upcb, struct pbuf *p, const ip_addr_t *addr, u16_t port);

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Initialize the server application.
  * @param  None
  * @retval None
  */
void udp_echoserver_init(void)
{
   struct udp_pcb *udp_echo_server_pcb;
   err_t err;
   
#if (UDP_ECHO_SERVER_DEBUG)
   dmc_puts("udp_echoserver_init\n");
#endif

   /* Create a new UDP control block  */
   udp_echo_server_pcb = udp_new();
   
   if (udp_echo_server_pcb)
   {
     /* Bind the udp_echo_server_pcb to the UDP_PORT port */
     /* Using IP_ADDR_ANY allow the udp_echo_server_pcb to be used by any local interface */
      err = udp_bind(udp_echo_server_pcb, IP_ADDR_ANY, UDP_SERVER_PORT);

#if (UDP_ECHO_SERVER_DEBUG)
    dmc_putint((udp_echo_server_pcb->remote_ip.addr & 0xff));
    dmc_putc('.');
    dmc_putint((udp_echo_server_pcb->remote_ip.addr & 0xff00) >> 8);
    dmc_putc('.');
    dmc_putint((udp_echo_server_pcb->remote_ip.addr & 0xff0000) >> 16);
    dmc_putc('.');
    dmc_putint((udp_echo_server_pcb->remote_ip.addr & 0xff000000) >> 24);
    dmc_putc(':');
    dmc_putint(udp_echo_server_pcb->remote_port);
    dmc_puts(" -> ");
    dmc_putint((udp_echo_server_pcb->local_ip.addr & 0xff));
    dmc_putc('.');
    dmc_putint((udp_echo_server_pcb->local_ip.addr & 0xff00) >> 8);
    dmc_putc('.');
    dmc_putint((udp_echo_server_pcb->local_ip.addr & 0xff0000) >> 16);
    dmc_putc('.');
    dmc_putint((udp_echo_server_pcb->local_ip.addr & 0xff000000) >> 24);
    dmc_putc(':');
    dmc_putint(udp_echo_server_pcb->local_port);
    dmc_putc('\n');
#endif

      if(err == ERR_OK)
      {
        /* Set a receive callback for the udp_echo_server_pcb */
        udp_recv(udp_echo_server_pcb, udp_echoserver_receive_callback, NULL);
      }
      else
      {
        udp_remove(udp_echo_server_pcb);
      }
   }
}

/**
  * @brief This function is called when an UDP datagrm has been received on the port UDP_PORT.
  * @param arg user supplied argument (udp_pcb.recv_arg)
  * @param pcb the udp_pcb which received data
  * @param p the packet buffer that was received
  * @param addr the remote IP address from which the packet was received
  * @param port the remote port from which the packet was received
  * @retval None
  */
void udp_echoserver_receive_callback(void *arg, struct udp_pcb *upcb, struct pbuf *p, const ip_addr_t *addr, u16_t port)
{
  struct pbuf *p_tx;
  
#if (UDP_ECHO_SERVER_DEBUG)
  dmc_puts("udp_echoserver_receive_callback\n");
#endif

  /* Allocate pbuf from RAM */
  p_tx = pbuf_alloc(PBUF_TRANSPORT, p->len, PBUF_RAM);
  
  if(p_tx != NULL)
  {
#if (UDP_ECHO_SERVER_DEBUG)
    dmc_putint((upcb->remote_ip.addr & 0xff));
    dmc_putc('.');
    dmc_putint((upcb->remote_ip.addr & 0xff00) >> 8);
    dmc_putc('.');
    dmc_putint((upcb->remote_ip.addr & 0xff0000) >> 16);
    dmc_putc('.');
    dmc_putint((upcb->remote_ip.addr & 0xff000000) >> 24);
    dmc_putc(':');
    dmc_putint(upcb->remote_port);
    dmc_puts(" -> ");
    dmc_putint((upcb->local_ip.addr & 0xff));
    dmc_putc('.');
    dmc_putint((upcb->local_ip.addr & 0xff00) >> 8);
    dmc_putc('.');
    dmc_putint((upcb->local_ip.addr & 0xff0000) >> 16);
    dmc_putc('.');
    dmc_putint((upcb->local_ip.addr & 0xff000000) >> 24);
    dmc_putc(':');
    dmc_putint(upcb->local_port);
    dmc_putc('\n');

    dmc_putint((addr->addr & 0xff));
    dmc_putc('.');
    dmc_putint((addr->addr & 0xff00) >> 8);
    dmc_putc('.');
    dmc_putint((addr->addr & 0xff0000) >> 16);
    dmc_putc('.');
    dmc_putint((addr->addr & 0xff000000) >> 24);
    dmc_putc(':');
    dmc_putint(port);
    dmc_putc('\n');

    dmc_putslen(p->payload, p->len);
    dmc_putcr();
#endif

    dmc_swap_case_len(p->payload, p->len);

    /* Copy data to pbuf */
    pbuf_take(p_tx, (char*)p->payload, p->len);

    /* Connect to the remote client */
    udp_connect(upcb, addr, UDP_CLIENT_PORT);
    
    /* Tell the client that we have accepted it */
    udp_send(upcb, p_tx);
    
    /* Free the UDP connection, so we can accept new clients */
    udp_disconnect(upcb);
    
    /* Free the p_tx buffer */
    pbuf_free(p_tx);
  }
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
