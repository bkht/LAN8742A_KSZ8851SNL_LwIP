/**
  ******************************************************************************
  * @file    LwIP/LwIP_TCP_Echo_Client/Src/tcp_echoclient.c
  * @author  MCD Application Team
  * @brief   tcp echoclient application using LwIP RAW API
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
#include "lwip/debug.h"
#include "lwip/stats.h"
#include "lwip/tcp.h"
#include "lwip/memp.h"
#include <stdio.h>
#include <string.h>
#include "tcp_echoclient.h"

#define TCP_ECHO_CLIENT_DEBUG                 0

/*Static DEST IP ADDRESS: DEST_IP_ADDR0.DEST_IP_ADDR1.DEST_IP_ADDR2.DEST_IP_ADDR3 */
#define DEST_IP_ADDR0   ((uint8_t)192U)
#define DEST_IP_ADDR1   ((uint8_t)168U)
#define DEST_IP_ADDR2   ((uint8_t)25U)
#define DEST_IP_ADDR3   ((uint8_t)148U)

//#define DEST_PORT       ((uint16_t)53831U)
#define DEST_PORT       ((uint16_t)502U)

#define NETIF_NAME                            "m1"


#if LWIP_TCP
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
u8_t  recev_buf[50];
__IO uint32_t tcp_ec_message_count=0;

u8_t   ec_data[100];

struct tcp_pcb *tcp_echo_client_pcb;

/* ECHO protocol states */
enum echoclient_states
{
  ES_NOT_CONNECTED = 0,
  ES_CONNECTED,
  ES_RECEIVED,
  ES_CLOSING,
};

/* structure to be passed as argument to the tcp callbacks */
struct echoclient
{
  enum echoclient_states state; /* connection status */
  struct tcp_pcb *pcb;          /* pointer on the current tcp_pcb */
  struct pbuf *p_tx;            /* pointer on pbuf to be transmitted */
};

/* Private function prototypes -----------------------------------------------*/
static err_t tcp_echoclient_recv(void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err);
static void tcp_echoclient_connection_close(struct tcp_pcb *tpcb, struct echoclient * es);
static err_t tcp_echoclient_poll(void *arg, struct tcp_pcb *tpcb);
static err_t tcp_echoclient_sent(void *arg, struct tcp_pcb *tpcb, u16_t len);
static void tcp_echoclient_send(struct tcp_pcb *tpcb, struct echoclient * es);
static err_t tcp_echoclient_connected(void *arg, struct tcp_pcb *tpcb, err_t err);

/* Private functions ---------------------------------------------------------*/


/**
  * @brief  Connects to the TCP echo server
  * @param  None
  * @retval None
  */
void tcp_echoclient_connect(void)
{
  ip_addr_t DestIPaddr;
  
  ip4_addr_t ipaddr;
  ipaddr.addr = 192 | (168 << 8) | (25 << 16) || (232 << 24);
  ip4_addr_t netmask;
  ipaddr.addr = 255 | (255 << 8) | (255 << 16) || (0 << 24);
  ip4_addr_t gw;
  ipaddr.addr = 192 | (168 << 8) | (25 << 16) || (253 << 24);

//  netif_set_addr(&gnetif0, &ipaddr, &netmask, &gw);

  /* create new tcp pcb */
  tcp_echo_client_pcb = tcp_new();
  
  if (tcp_echo_client_pcb != NULL)
  {
    IP4_ADDR( &DestIPaddr, DEST_IP_ADDR0, DEST_IP_ADDR1, DEST_IP_ADDR2, DEST_IP_ADDR3 );

//    tcp_echo_client_pcb->local_ip.addr = 192 | (168 << 8) | (25 << 16) || (232 << 24);


    /* from a specified netif connect to destination address/port */
    const char netif_name[] = NETIF_NAME;
    tcp_connect_by_name(netif_name, tcp_echo_client_pcb, &DestIPaddr, DEST_PORT, tcp_echoclient_connected);

#if (TCP_ECHO_CLIENT_DEBUG)
    dmc_puts("tcp_echoclient_connect\n");
    dmc_putint((tcp_echo_client_pcb->local_ip.addr & 0xff));
    dmc_putc('.');
    dmc_putint((tcp_echo_client_pcb->local_ip.addr & 0xff00) >> 8);
    dmc_putc('.');
    dmc_putint((tcp_echo_client_pcb->local_ip.addr & 0xff0000) >> 16);
    dmc_putc('.');
    dmc_putint((tcp_echo_client_pcb->local_ip.addr & 0xff000000) >> 24);
    dmc_putc(':');
    dmc_putint(tcp_echo_client_pcb->local_port);
    dmc_puts(" -> ");
    dmc_putint((tcp_echo_client_pcb->remote_ip.addr & 0xff));
    dmc_putc('.');
    dmc_putint((tcp_echo_client_pcb->remote_ip.addr & 0xff00) >> 8);
    dmc_putc('.');
    dmc_putint((tcp_echo_client_pcb->remote_ip.addr & 0xff0000) >> 16);
    dmc_putc('.');
    dmc_putint((tcp_echo_client_pcb->remote_ip.addr & 0xff000000) >> 24);
    dmc_putc(':');
    dmc_putint(tcp_echo_client_pcb->remote_port);
    dmc_putc('\n');
#endif
  }
  else
  {
    /* deallocate the pcb */
    memp_free(MEMP_TCP_PCB, tcp_echo_client_pcb);
#ifdef SERIAL_DEBUG
    printf("\n\r can not create tcp pcb");
#endif 
  }
}

/**
  * @brief Function called when TCP connection established
  * @param tpcb: pointer on the connection control block
  * @param err: when connection correctly established err should be ERR_OK 
  * @retval err_t: returned error 
  */
static err_t tcp_echoclient_connected(void *arg, struct tcp_pcb *tpcb, err_t err)
{
  struct echoclient *es = NULL;

#if (TCP_ECHO_CLIENT_DEBUG)
  dmc_puts("tcp_echoclient_connected\n");
  dmc_putint((tpcb->local_ip.addr & 0xff));
  dmc_putc('.');
  dmc_putint((tpcb->local_ip.addr & 0xff00) >> 8);
  dmc_putc('.');
  dmc_putint((tpcb->local_ip.addr & 0xff0000) >> 16);
  dmc_putc('.');
  dmc_putint((tpcb->local_ip.addr & 0xff000000) >> 24);
  dmc_putc(':');
  dmc_putint(tpcb->local_port);
  dmc_puts(" -> ");
  dmc_putint((tpcb->remote_ip.addr & 0xff));
  dmc_putc('.');
  dmc_putint((tpcb->remote_ip.addr & 0xff00) >> 8);
  dmc_putc('.');
  dmc_putint((tpcb->remote_ip.addr & 0xff0000) >> 16);
  dmc_putc('.');
  dmc_putint((tpcb->remote_ip.addr & 0xff000000) >> 24);
  dmc_putc(':');
  dmc_putint(tpcb->remote_port);
  dmc_putc('\n');
#endif

  if (err == ERR_OK)   
  {
    /* allocate structure es to maintain tcp connection informations */
    es = (struct echoclient *)mem_malloc(sizeof(struct echoclient));
  
    if (es != NULL)
    {
      es->state = ES_CONNECTED;
      es->pcb = tpcb;
      
      sprintf((char*)ec_data, "Sending TCP Client message %d", (int)tcp_ec_message_count);
      tcp_ec_message_count++;
        
      /* allocate pbuf */
      es->p_tx = pbuf_alloc(PBUF_TRANSPORT, strlen((char*)ec_data) , PBUF_POOL);
         
      if (es->p_tx)
      {       
//#if (TCP_ECHO_CLIENT_DEBUG)
        dmc_puts("TCP echo client send data\n");
        dmc_putscr(ec_data);
//#endif

        /* copy data to pbuf */
        pbuf_take(es->p_tx, (char*)ec_data, strlen((char*)ec_data));
        
        /* pass newly allocated es structure as argument to tpcb */
        tcp_arg(tpcb, es);
  
        /* initialize LwIP tcp_recv callback function */ 
        tcp_recv(tpcb, tcp_echoclient_recv);
  
        /* initialize LwIP tcp_sent callback function */
        tcp_sent(tpcb, tcp_echoclient_sent);
  
        /* initialize LwIP tcp_poll callback function */
        tcp_poll(tpcb, tcp_echoclient_poll, 1);
    
        /* send data */
        tcp_echoclient_send(tpcb, es);
        
        return ERR_OK;
      }
    }
    else
    {
      /* close connection */
#if (TCP_ECHO_CLIENT_DEBUG)
      dmc_puts("close connection\n");
#endif

      tcp_echoclient_connection_close(tpcb, es);
      
      /* return memory allocation error */
      return ERR_MEM;  
    }
  }
  else
  {
    /* close connection */
    tcp_echoclient_connection_close(tpcb, es);
  }
  return err;
}
    
/**
  * @brief tcp_receiv callback
  * @param arg: argument to be passed to receive callback 
  * @param tpcb: tcp connection control block 
  * @param err: receive error code 
  * @retval err_t: retuned error  
  */
static err_t tcp_echoclient_recv(void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err)
{ 
  struct echoclient *es;
  err_t ret_err; 

#if (TCP_ECHO_CLIENT_DEBUG)
  dmc_puts("tcp_echoclient_recv\n");
#endif

  LWIP_ASSERT("arg != NULL",arg != NULL);
  
  es = (struct echoclient *)arg;
  
  /* if we receive an empty tcp frame from server => close connection */
  if (p == NULL)
  {
    /* remote host closed connection */
#if (TCP_ECHO_CLIENT_DEBUG)
    dmc_puts("remote host closed connection\n");
#endif

    es->state = ES_CLOSING;
    if(es->p_tx == NULL)
    {
       /* we're done sending, close connection */
       tcp_echoclient_connection_close(tpcb, es);
    }
    else
    {    
      /* send remaining data*/
      tcp_echoclient_send(tpcb, es);
    }
    ret_err = ERR_OK;
  }   
  /* else : a non empty frame was received from echo server but for some reason err != ERR_OK */
  else if(err != ERR_OK)
  {
    /* free received pbuf */
#if (TCP_ECHO_CLIENT_DEBUG)
    dmc_puts("free received pbuf\n");
#endif

    if (p != NULL)
    {
      pbuf_free(p);
    }
    ret_err = err;
  }
  else if(es->state == ES_CONNECTED)
  {
    /* increment message count */

    tcp_ec_message_count++;

    if (p->len)
    {
      uint8_t local_ip[4];
      local_ip[0] = (tpcb->local_ip.addr & 0xff);
      local_ip[1] = (tpcb->local_ip.addr & 0xff00) >> 8;
      local_ip[2] = (tpcb->local_ip.addr & 0xff0000) >> 16;
      local_ip[3] = (tpcb->local_ip.addr & 0xff000000) >> 24;
      if (local_ip[3] == 232)
      {
        dmc_puts(TERMINAL_LIGHT_GREEN);
      }
      if (local_ip[3] == 233)
      {
        dmc_puts(TERMINAL_LIGHT_CYAN);
      }
      if (local_ip[3] == 234)
      {
        dmc_puts(TERMINAL_LIGHT_BLUE);
      }
      dmc_puts("Received ");
      dmc_putint(p->len);
      dmc_puts(" Bytes via ");
      dmc_putint(local_ip[0]);
      dmc_putc('.');
      dmc_putint(local_ip[1]);
      dmc_putc('.');
      dmc_putint(local_ip[2]);
      dmc_putc('.');
      dmc_putint(local_ip[3]);
      dmc_putc('\n');

//      for (uint16_t i = 0; i < p->len; i++)
//      {
//        uint8_t c = pbuf_get_at(p, i);
//        dmc_puthex2(c);
//        dmc_putc(' ');
//      }
//      dmc_putc('\n');
      for (uint16_t i = 0; i < p->len; i++)
      {
        uint8_t c = pbuf_get_at(p, i);
        if ((c >= ' ') && (c <= '~'))
        {
          dmc_putc(c);
        }
      }
      dmc_putc('\n');
      dmc_puts(TERMINAL_DEFAULT);
    }

    /* Acknowledge data reception */
    tcp_recved(tpcb, p->tot_len);  
    
    pbuf_free(p);
    tcp_echoclient_connection_close(tpcb, es);
    ret_err = ERR_OK;
  }
  /* data received when connection already closed */
  else
  {
    /* Acknowledge data reception */
#if (TCP_ECHO_CLIENT_DEBUG)
    dmc_puts("Acknowledge data reception\n");
#endif

    tcp_recved(tpcb, p->tot_len);
    
    /* free pbuf and do nothing */
    pbuf_free(p);
    ret_err = ERR_OK;
  }
  return ret_err;
}

/**
  * @brief function used to send data
  * @param  tpcb: tcp control block
  * @param  es: pointer on structure of type echoclient containing info on data 
  *             to be sent
  * @retval None 
  */
static void tcp_echoclient_send(struct tcp_pcb *tpcb, struct echoclient * es)
{
  struct pbuf *ptr;
  err_t wr_err = ERR_OK;

#if (TCP_ECHO_CLIENT_DEBUG)
  dmc_puts("tcp_echoclient_send\n");
#endif

  while ((wr_err == ERR_OK) &&
         (es->p_tx != NULL) && 
         (es->p_tx->len <= tcp_sndbuf(tpcb)))
  {
    
    /* get pointer on pbuf from es structure */
    ptr = es->p_tx;

    /* enqueue data for transmission */
    wr_err = tcp_write(tpcb, ptr->payload, ptr->len, 1);
    
    if (wr_err == ERR_OK)
    { 
      /* continue with next pbuf in chain (if any) */
      es->p_tx = ptr->next;
      
      if(es->p_tx != NULL)
      {
        /* increment reference count for es->p */
        pbuf_ref(es->p_tx);
      }
      
      /* free pbuf: will free pbufs up to es->p (because es->p has a reference count > 0) */
      pbuf_free(ptr);
   }
   else if(wr_err == ERR_MEM)
   {
      /* we are low on memory, try later, defer to poll */
     es->p_tx = ptr;
   }
   else
   {
     /* other problem ?? */
   }
  }
}

/**
  * @brief  This function implements the tcp_poll callback function
  * @param  arg: pointer on argument passed to callback
  * @param  tpcb: tcp connection control block
  * @retval err_t: error code
  */
static err_t tcp_echoclient_poll(void *arg, struct tcp_pcb *tpcb)
{
  err_t ret_err;
  struct echoclient *es;

  es = (struct echoclient*)arg;
  if (es != NULL)
  {
#if (TCP_ECHO_CLIENT_DEBUG)
    dmc_puts("tcp_echoclient_poll\n");
#endif

    if (es->p_tx != NULL)
    {
      /* there is a remaining pbuf (chain) , try to send data */
      tcp_echoclient_send(tpcb, es);
    }
    else
    {
      /* no remaining pbuf (chain)  */
      if(es->state == ES_CLOSING)
      {
        /* close tcp connection */
        tcp_echoclient_connection_close(tpcb, es);
      }
    }
    ret_err = ERR_OK;
  }
  else
  {
    /* nothing to be done */
    tcp_abort(tpcb);
    ret_err = ERR_ABRT;
  }
  return ret_err;
}

/**
  * @brief  This function implements the tcp_sent LwIP callback (called when ACK
  *         is received from remote host for sent data) 
  * @param  arg: pointer on argument passed to callback
  * @param  tcp_pcb: tcp connection control block
  * @param  len: length of data sent 
  * @retval err_t: returned error code
  */
static err_t tcp_echoclient_sent(void *arg, struct tcp_pcb *tpcb, u16_t len)
{
  struct echoclient *es;

  LWIP_UNUSED_ARG(len);

  es = (struct echoclient *)arg;

#if (TCP_ECHO_CLIENT_DEBUG)
  dmc_puts("tcp_echoclient_sent\n");
#endif

  if(es->p_tx != NULL)
  {
    /* still got pbufs to send */
    tcp_echoclient_send(tpcb, es);
  }

  return ERR_OK;
}

/**
  * @brief This function is used to close the tcp connection with server
  * @param tpcb: tcp connection control block
  * @param es: pointer on echoclient structure
  * @retval None
  */
static void tcp_echoclient_connection_close(struct tcp_pcb *tpcb, struct echoclient * es )
{
#if (TCP_ECHO_CLIENT_DEBUG)
  dmc_puts("tcp_echoclient_connection_close\n");
#endif

  /* remove callbacks */
  tcp_recv(tpcb, NULL);
  tcp_sent(tpcb, NULL);
  tcp_poll(tpcb, NULL,0);

  if (es != NULL)
  {
    mem_free(es);
  }

  /* close tcp connection */
  tcp_close(tpcb);  
}

#endif /* LWIP_TCP */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
