/**
 * Copyright (c) 2001-2004 Swedish Institute of Computer Science.
 * All rights reserved. 
 * 
 * Redistribution and use in source and binary forms, with or without modification, 
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote products
 *    derived from this software without specific prior written permission. 
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR IMPLIED 
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF 
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT 
 * SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, 
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT 
 * OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS 
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING 
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY 
 * OF SUCH DAMAGE.
 *
 * This file is part of and a contribution to the lwIP TCP/IP stack.
 *
 * Credits go to Adam Dunkels (and the current maintainers) of this software.
 *
 * Christiaan Simons rewrote this file to get a more stable echo application.
 *
 **/

 /* This file was modified by ST */

#include "tcp_echoserver.h"
#include "lwip/debug.h"
#include "lwip/stats.h"
#include "lwip/tcp.h"

#define TCP_ECHO_SERVER_DEBUG                 1

#define SERVER_PORT 5005

#if LWIP_TCP

static struct tcp_pcb *tcp_echo_server_pcb;

/* ECHO protocol states */
enum tcp_echoserver_states
{
  ES_NONE = 0,
  ES_ACCEPTED,
  ES_RECEIVED,
  ES_CLOSING
};

/* structure for maintaing connection infos to be passed as argument 
   to LwIP callbacks*/
struct tcp_echoserver_struct
{
  u8_t state;             /* current connection state */
  u8_t retries;
  struct tcp_pcb *pcb;    /* pointer on the current tcp_pcb */
  struct pbuf *p;         /* pointer on the received/to be transmitted pbuf */
};

static err_t tcp_echoserver_accept(void *arg, struct tcp_pcb *newpcb, err_t err);
static err_t tcp_echoserver_recv(void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err);
static void tcp_echoserver_error(void *arg, err_t err);
static err_t tcp_echoserver_poll(void *arg, struct tcp_pcb *tpcb);
static err_t tcp_echoserver_sent(void *arg, struct tcp_pcb *tpcb, u16_t len);
static void tcp_echoserver_send(struct tcp_pcb *tpcb, struct tcp_echoserver_struct *es);
static void tcp_echoserver_connection_close(struct tcp_pcb *tpcb, struct tcp_echoserver_struct *es);

/**
  * @brief  Initializes the tcp echo server
  * @param  None
  * @retval None
  */
void tcp_echoserver_init(void)
{
  /* LwIP API calls tcp_new to allocate a new TCP protocol control block (PCB)
   * (tcp_echo_server_pcb).
   */
  /* Creates a new TCP PCB (protocol control block). */
  /* create new tcp pcb */
  tcp_echo_server_pcb = tcp_new();

  if (tcp_echo_server_pcb != NULL)
  {
    err_t err;
    /* The allocated TCP PCB is bound to a local IP address and port using tcp_bind
     * function.
     */
    /* Binds a TCP PCB to a local IP address and port. */
    /* bind echo_pcb to port 7 (ECHO protocol) */
    err = tcp_bind(tcp_echo_server_pcb, IP_ADDR_ANY, SERVER_PORT);
    
    if (err == ERR_OK)
    {
      /* After binding the TCP PCB, tcp_listen function is called in order to start the TCP
       * listening process on the TCP PCB.
       */
      /* start tcp listening for echo_pcb */
      tcp_echo_server_pcb = tcp_listen(tcp_echo_server_pcb);

      /* Finally a tcp_echoserver_accept callback function should be assigned to handle
       * incoming TCP connections on the TCP PCB. This is done using tcp_accept LwIP API
       * function.
       */
      /* Assigns a callback function that will be called when a new TCP connection arrives. */
      /* initialize LwIP tcp_accept callback function */
      tcp_accept(tcp_echo_server_pcb, tcp_echoserver_accept);
      /* Starting from this point, the TCP server is ready to accept any incoming connection
       * from remote clients.
       */
    }
    else 
    {
      /* abort? output diagnostic? */
      printf("Can not bind pcb\n");
      /* deallocate the pcb */
      memp_free(MEMP_TCP_PCB, tcp_echo_server_pcb);
    }
  }
  else
  {
    /* abort? output diagnostic? */
    printf("Can not create new pcb\n");
  }
}

/**
  * @brief  This function is the implementation of tcp_accept LwIP callback
  * @param  arg: not used
  * @param  newpcb: pointer on tcp_pcb struct for the newly created tcp connection
  * @param  err: not used 
  * @retval err_t: error status
  */
static err_t tcp_echoserver_accept(void *arg, struct tcp_pcb *newpcb, err_t err)
{
  err_t ret_err;
  struct tcp_echoserver_struct *es;

  LWIP_UNUSED_ARG(arg);
  LWIP_UNUSED_ARG(err);

  /* The new TCP connection is passed to tcp_echoserver_accept callback function
   * through newpcb parameter.
   */

//  dmc_puts("tcp_echoserver_accept\n");

  /* set priority for the newly accepted tcp connection newpcb */
  tcp_setprio(newpcb, TCP_PRIO_MIN);

  /* An es structure is used to maintain the application status. It is passed as an argument
   * to the TCP PCB “newpcb” connection by calling tcp_arg LwIP API.
   */

  /* allocate structure es to maintain tcp connection informations */
  es = (struct tcp_echoserver_struct *)mem_malloc(sizeof(struct tcp_echoserver_struct));
  if (es != NULL)
  {
    es->state = ES_ACCEPTED;
    es->pcb = newpcb;
    es->retries = 0;
    es->p = NULL;
    
    /* pass newly allocated es structure as argument to newpcb */
    tcp_arg(newpcb, es);
    
    /* A TCP receive callback function, tcp_echoserver_recv, is assigned by calling LwIP API
     * tcp_recv. This callback will handle all the data traffic with the remote client.
     */
    /* initialize lwip tcp_recv callback function for newpcb  */ 
    tcp_recv(newpcb, tcp_echoserver_recv);
    /* A TCP error callback function, tcp_echoserver_error, is assigned by calling LwIP API
     * tcp_err .This callback will handle TCP errors.
     */
    /* initialize lwip tcp_err callback function for newpcb  */
    tcp_err(newpcb, tcp_echoserver_error);
    /* A TCP poll callback function, tcp_echoserver_poll, is assigned by calling LwIP API
     * tcp_poll to handle periodic application tasks (such as checking if the application data
     * remains to be transmitted).
    */
    /* initialize lwip tcp_poll callback function for newpcb */
    tcp_poll(newpcb, tcp_echoserver_poll, 0);
    
    ret_err = ERR_OK;
  }
  else
  {
    /*  close tcp connection */
    tcp_echoserver_connection_close(newpcb, es);
    /* return memory error */
    ret_err = ERR_MEM;
  }
  return ret_err;  
}


/**
  * @brief  This function is the implementation for tcp_recv LwIP callback
  * @param  arg: pointer on a argument for the tcp_pcb connection
  * @param  tpcb: pointer on the tcp_pcb connection
  * @param  pbuf: pointer on the received pbuf
  * @param  err: error information regarding the reveived pbuf
  * @retval err_t: error code
  */
static err_t tcp_echoserver_recv(void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err)
{
  struct tcp_echoserver_struct *es;
  err_t ret_err;

  LWIP_ASSERT("arg != NULL",arg != NULL);
  
//  dmc_puts("tcp_echoserver_recv\n");

  es = (struct tcp_echoserver_struct *)arg;
  
  /* if we receive an empty tcp frame from client => close connection */
  if (p == NULL)
  {
    /* remote host closed connection */
    es->state = ES_CLOSING;
    if(es->p == NULL)
    {
       /* we're done sending, close connection */
       tcp_echoserver_connection_close(tpcb, es);
    }
    else
    {
      /* we're not done yet */
      /* acknowledge received packet */
      tcp_sent(tpcb, tcp_echoserver_sent);
      
      /* send remaining data*/
      tcp_echoserver_send(tpcb, es);
    }
    ret_err = ERR_OK;
  }   
  /* else : a non empty frame was received from client but for some reason err != ERR_OK */
  else if(err != ERR_OK)
  {
    /* cleanup, for unkown reason */
    /* free received pbuf*/
    if (p != NULL)
    {
      es->p = NULL;
      pbuf_free(p);
    }
    ret_err = err;
  }
  else if(es->state == ES_ACCEPTED)
  {
    /* first data chunk in p->payload */
    es->state = ES_RECEIVED;
    
    /* store reference to incoming pbuf (chain) */
    es->p = p;
    
    /* install send completion notifier */

    /* initialize LwIP tcp_sent callback function */
    tcp_sent(tpcb, tcp_echoserver_sent);
    
    /* send back the received data (echo) */
    tcp_echoserver_send(tpcb, es);
    
    ret_err = ERR_OK;
  }
  else if (es->state == ES_RECEIVED)
  {
    /* read some more data */
    /* more data received from client and previous data has been already sent */
    if(es->p == NULL)
    {
      es->p = p;
  
//      dmc_puts("buffer:\n");
//      dmc_puts(p);
      /* send back received data */
      tcp_echoserver_send(tpcb, es);
    }
    else
    {
      struct pbuf *ptr;

      /* chain pbufs to the end of what we recv'ed previously */
      ptr = es->p;
      pbuf_chain(ptr,p);
    }
    ret_err = ERR_OK;
  }
  else if(es->state == ES_CLOSING)
  {
    /* odd case, remote side closing twice, trash data */
    tcp_recved(tpcb, p->tot_len);
    es->p = NULL;
    pbuf_free(p);
    ret_err = ERR_OK;
  }
  else
  {
    /* unkown es->state, trash data */
    tcp_recved(tpcb, p->tot_len);
    es->p = NULL;
    pbuf_free(p);
    ret_err = ERR_OK;
  }
  return ret_err;
}

/**
  * @brief  This function implements the tcp_err callback function (called
  *         when a fatal tcp_connection error occurs. 
  * @param  arg: pointer on argument parameter 
  * @param  err: not used
  * @retval None
  */
static void tcp_echoserver_error(void *arg, err_t err)
{
  struct tcp_echoserver_struct *es;

  LWIP_UNUSED_ARG(err);

//  dmc_puts("tcp_echoserver_error\n");

  es = (struct tcp_echoserver_struct *)arg;
  if (es != NULL)
  {
    /*  free es structure */
    mem_free(es);
  }
}

/**
  * @brief  This function implements the tcp_poll LwIP callback function
  * @param  arg: pointer on argument passed to callback
  * @param  tpcb: pointer on the tcp_pcb for the current tcp connection
  * @retval err_t: error code
  */
static err_t tcp_echoserver_poll(void *arg, struct tcp_pcb *tpcb)
{
  err_t ret_err;
  struct tcp_echoserver_struct *es;

//  dmc_puts("tcp_echoserver_poll\n");

  es = (struct tcp_echoserver_struct *)arg;
  if (es != NULL)
  {
    if (es->p != NULL)
    {
      tcp_sent(tpcb, tcp_echoserver_sent);
      /* there is a remaining pbuf (chain) , try to send data */
      tcp_echoserver_send(tpcb, es);
    }
    else
    {
      /* no remaining pbuf (chain)  */
      if(es->state == ES_CLOSING)
      {
        /*  close tcp connection */
        tcp_echoserver_connection_close(tpcb, es);
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
  * @param  None
  * @retval None
  */
static err_t tcp_echoserver_sent(void *arg, struct tcp_pcb *tpcb, u16_t len)
{
  struct tcp_echoserver_struct *es;

//  dmc_puts("tcp_echoserver_sent\n");

  LWIP_UNUSED_ARG(len);

  es = (struct tcp_echoserver_struct *)arg;
  es->retries = 0;
  
  if(es->p != NULL)
  {
    /* still got pbufs to send */
    tcp_sent(tpcb, tcp_echoserver_sent);
    tcp_echoserver_send(tpcb, es);
  }
  else
  {
    /* no more pbufs to send */
    /* if no more data to send and client closed connection*/
    if(es->state == ES_CLOSING)
    {
      tcp_echoserver_connection_close(tpcb, es);
    }
  }
  return ERR_OK;
}


/**
  * @brief  This function is used to send data for tcp connection
  * @param  tpcb: pointer on the tcp_pcb connection
  * @param  es: pointer on echo_state structure
  * @retval None
  */
static void tcp_echoserver_send(struct tcp_pcb *tpcb, struct tcp_echoserver_struct *es)
{
  struct pbuf *ptr;
  err_t wr_err = ERR_OK;
 
//  dmc_puts("tcp_echoserver_send\n");

  while ((wr_err == ERR_OK) &&
         (es->p != NULL) && 
         (es->p->len <= tcp_sndbuf(tpcb)))
  {
    
    /* get pointer on pbuf from es structure */
    ptr = es->p;

#if (TCP_ECHO_SERVER_DEBUG)
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
    dmc_putint((tpcb->remote_ip.addr & 0xff));
    dmc_putc('.');
    dmc_putint((tpcb->remote_ip.addr & 0xff00) >> 8);
    dmc_putc('.');
    dmc_putint((tpcb->remote_ip.addr & 0xff0000) >> 16);
    dmc_putc('.');
    dmc_putint((tpcb->remote_ip.addr & 0xff000000) >> 24);
    //    dmc_putc(':');
    //    dmc_putint(tpcb->remote_port);
    dmc_puts(" -> ");
    dmc_putint((tpcb->local_ip.addr & 0xff));
    dmc_putc('.');
    dmc_putint((tpcb->local_ip.addr & 0xff00) >> 8);
    dmc_putc('.');
    dmc_putint((tpcb->local_ip.addr & 0xff0000) >> 16);
    dmc_putc('.');
    dmc_putint((tpcb->local_ip.addr & 0xff000000) >> 24);
    dmc_putc(':');
    dmc_putint(tpcb->local_port);
    dmc_putc('\n');

    //	dmc_puts("Payload: ");
    dmc_putslen(ptr->payload, ptr->len);
    dmc_putcr();
    dmc_swap_case_len(ptr->payload, ptr->len);
    //	dmc_putslen(ptr->payload, ptr->len);
    //	dmc_putcr();
    dmc_puts(TERMINAL_DEFAULT);
#endif

    /* enqueue data for transmission */
    wr_err = tcp_write(tpcb, ptr->payload, ptr->len, 1);

    if (wr_err == ERR_OK)
    {
      u16_t plen;
      u8_t freed;

      plen = ptr->len;

      /* continue with next pbuf in chain (if any) */
      es->p = ptr->next;

      if(es->p != NULL)
      {
        /* new reference! */
        /* increment reference count for es->p */
        pbuf_ref(es->p);
      }

      /* chop first pbuf from chain */
      do
      {
        /* try hard to free pbuf */
        freed = pbuf_free(ptr);
      }
      while(freed == 0);
      /* we can read more data now */
      tcp_recved(tpcb, plen);
    }
    else if(wr_err == ERR_MEM)
    {
      /* we are low on memory, try later / harder, defer to poll */
      es->p = ptr;
    }
    else
    {
      /* other problem ?? */
    }
  }
}

/**
  * @brief  This functions closes the tcp connection
  * @param  tcp_pcb: pointer on the tcp connection
  * @param  es: pointer on echo_state structure
  * @retval None
  */
static void tcp_echoserver_connection_close(struct tcp_pcb *tpcb, struct tcp_echoserver_struct *es)
{
//	  dmc_puts("tcp_echoserver_connection_close\n");

  /* remove all callbacks */
  tcp_arg(tpcb, NULL);
  tcp_sent(tpcb, NULL);
  tcp_recv(tpcb, NULL);
  tcp_err(tpcb, NULL);
  tcp_poll(tpcb, NULL, 0);
  
  /* delete es structure */
  if (es != NULL)
  {
    mem_free(es);
  }  
  
  /* close tcp connection */
  tcp_close(tpcb);
}

#endif /* LWIP_TCP */
