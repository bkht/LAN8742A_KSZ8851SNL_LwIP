/**
 * @file
 * Ethernet Interface Skeleton
 *
 */

/*
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
 * This file is part of the lwIP TCP/IP stack.
 * 
 * Author: Adam Dunkels <adam@sics.se>
 *
 */

/*
 * This file is a skeleton for developing Ethernet network interface
 * drivers for lwIP. Add code to the low_level functions and do a
 * search-and-replace for the word "ethernetif" to replace it with
 * something that better describes your network interface.
 */

#include <dmc_print.h>
#include <KSZ8851SNL_1.h>
#include "lwip/opt.h"

//#if 0 /* don't build, this is only a skeleton, see previous comment */

#include "lwip/def.h"
#include "lwip/mem.h"
#include "lwip/pbuf.h"
#include "lwip/sys.h"
#include <lwip/stats.h>
#include <lwip/snmp.h>
#include <string.h>
#include "netif/etharp.h"
//#include "netif/ppp_oe.h"
#include "ethernetif_ksz8851_1.h"

/* Define those to better describe your network interface. */
#define KSZ8851_IFNAME0_1 'm'
#define KSZ8851_IFNAME1_1 '2'

#define KSZ8851_NET_MTU_1               1500

#define KSZ8851_DEBUG_1                 0
#define KSZ8851_DEBUG_IN_1              0
#define KSZ8851_DEBUG_OUT_1             0

uint8_t lwip_buf_1[KSZ8851_NET_MTU_1 * 2];

/**
 * MAC address to use.
 */
static uint8_t gs_uc_mac_address_KSZ8851_1[] =
{
  KSZ8851_ETHERNET_CONF_ETHADDR0_1,
  KSZ8851_ETHERNET_CONF_ETHADDR1_1,
  KSZ8851_ETHERNET_CONF_ETHADDR2_1,
  KSZ8851_ETHERNET_CONF_ETHADDR3_1,
  KSZ8851_ETHERNET_CONF_ETHADDR4_1,
  KSZ8851_ETHERNET_CONF_ETHADDR5_1
};

/**
 * Helper struct to hold private data used to operate your ethernet interface.
 * Keeping the ethernet address of the MAC in this struct is not necessary
 * as it is already kept in the struct netif.
 * But this is only an example, anyway...
 */
struct ethernetif_KSZ8851_1 {
  struct eth_addr *ethaddr;
  /* Add whatever per-interface state that is needed here. */
};

/* Forward declarations. */
//static void  ethernetif_input_KSZ8851(struct netif *netif);

/**
 * In this function, the hardware should be initialized.
 * Called from ethernetif_init().
 *
 * @param netif the already initialized lwip network interface structure
 *        for this ethernetif
 */
static void low_level_init_KSZ8851_1(struct netif *netif)
{
//  struct ethernetif *ethernetif = netif->state;
  
  dmc_puts("low_level_init_E1\n");

  /* set MAC hardware address length */
  netif->hwaddr_len = ETHARP_HWADDR_LEN;

  /* set MAC hardware address */
  netif->hwaddr[0] = gs_uc_mac_address_KSZ8851_1[0];
  netif->hwaddr[1] = gs_uc_mac_address_KSZ8851_1[1];
  netif->hwaddr[2] = gs_uc_mac_address_KSZ8851_1[2];
  netif->hwaddr[3] = gs_uc_mac_address_KSZ8851_1[3];
  netif->hwaddr[4] = gs_uc_mac_address_KSZ8851_1[4];
  netif->hwaddr[5] = gs_uc_mac_address_KSZ8851_1[5];

  /* maximum transfer unit */
  netif->mtu = KSZ8851_NET_MTU_1;
  
  if(!ksz8851_init_1())     // KSZ8851 Initialization
  {
#if (KSZ8851_DEBUG_1)
    dmc_puts(TERMINAL_LIGHT_RED);
    dmc_puts("low_level_init_E1 FAILED!\n");
    dmc_puts(TERMINAL_DEFAULT);
#endif
    return;   // Underlying network interface error
  }

//  HAL_Delay(1000);
#if (KSZ8851_DEBUG_1)
  dmc_puts(TERMINAL_LIGHT_GREEN);
  dmc_puts("low_level_init_E1 OK\n");
  dmc_puts(TERMINAL_DEFAULT);
#endif

//  ksz8851_IntDisable_1();
  ksz8851_IntEnable_1();
  ksz8851snl_reset_tx_1();
  ksz8851snl_reset_rx_1();
  ksz8851_IntClearAll_1();
  ksz8851_IntEnable_1();

  /* device capabilities */
  /* don't set NETIF_FLAG_ETHARP if this device is not an ethernet one */
  netif->flags = NETIF_FLAG_BROADCAST | NETIF_FLAG_ETHARP | NETIF_FLAG_LINK_UP;
 
  /* Do whatever else is needed to initialize interface. */  
}

/**
 * This function should do the actual transmission of the packet. The packet is
 * contained in the pbuf that is passed to the function. This pbuf
 * might be chained.
 *
 * @param netif the lwip network interface structure for this ethernetif
 * @param p the MAC packet to send (e.g. IP packet including MAC addresses and type)
 * @return ERR_OK if the packet could be sent
 *         an err_t value if the packet couldn't be sent
 *
 * @note Returning ERR_MEM here if a DMA queue of your MAC is full can lead to
 *       strange results. You might consider waiting for space in the DMA queue
 *       to become availale since the stack doesn't retry to send a packet
 *       dropped because of memory failure (except for the TCP timers).
 */

static err_t low_level_output_KSZ8851_1(struct netif *netif, struct pbuf *p)
{
  //struct ethernetif *ethernetif = netif->state;
  struct pbuf *q;
  int tx_len = 0;

//  initiate transfer();
  
  // Jack 2019-03-28 debug
#if (KSZ8851_DEBUG_OUT_1)
  dmc_puts(TERMINAL_GREEN);
  dmc_puts("low_level_output_E1 (M2)\n");
  dmc_puts(TERMINAL_DEFAULT);
#endif

#if ETH_PAD_SIZE
  pbuf_header(p, -ETH_PAD_SIZE); /* drop the padding word */
#endif

  for(q = p; q != NULL; q = q->next) {
    /* Send the data from the pbuf to the interface, one pbuf at a
       time. The size of the data in each pbuf is kept in the ->len
       variable. */
    /*send data from(q->payload, q->len);*/

    // Note we need 5 bytes space on the head
    memcpy((u8_t *)&lwip_buf_1[tx_len], (u8_t *)q->payload, q->len);
    tx_len += q->len;

    // Validation check to prevent data overflow
    if(tx_len > 1500 || tx_len > p->tot_len)
    {
      LWIP_PLATFORM_DIAG(("low_level_output_KSZ8851: error,tmplen=%"U32_F",tot_len=%"U32_F"\n\t", tx_len, p->tot_len));
      return ERR_BUF;
    }

  }

  // Jack Note we need 5 bytes space on the head
//  tx_len += 5;

#if (KSZ8851_DEBUG_OUT_1)
  uint8_t MAC_ST[] = { 0x00, 0x80, 0xe1, 0x00, 0x00, 0x02 };
  uint8_t MAC_PC[] = { 0xb4, 0xb6, 0x86, 0x8e, 0x57, 0xbf };
  uint8_t mark = 0;
  if (memcmp((u8_t *)&lwip_buf_1[0], MAC_ST, 6) == 0)
  {
    mark = 1;
  }
  if (memcmp((u8_t *)&lwip_buf_1[6], MAC_ST, 6) == 0)
  {
    mark = 1;
  }
  if (memcmp((u8_t *)&lwip_buf_1[0], MAC_PC, 6) == 0)
  {
    mark = 1;
  }
  if (memcmp((u8_t *)&lwip_buf_1[6], MAC_PC, 6) == 0)
  {
    mark = 1;
  }

  dmc_puts(TERMINAL_MAGENTA);
  dmc_puts("E1 M2 ");
  dmc_putc(netif->name[0]);
  dmc_putc(netif->name[1]);
  dmc_putc(' ');
  dmc_puthex2(netif->hwaddr[0]);
  dmc_putc(':');
  dmc_puthex2(netif->hwaddr[1]);
  dmc_putc(':');
  dmc_puthex2(netif->hwaddr[2]);
  dmc_putc(':');
  dmc_puthex2(netif->hwaddr[3]);
  dmc_putc(':');
  dmc_puthex2(netif->hwaddr[4]);
  dmc_putc(':');
  dmc_puthex2(netif->hwaddr[5]);
  dmc_putc(' ');
  dmc_putint(netif->ip_addr.addr & 0xff);
  dmc_putc('.');
  dmc_putint((netif->ip_addr.addr & 0xff00) >> 8);
  dmc_putc('.');
  dmc_putint((netif->ip_addr.addr & 0xff0000) >> 16);
  dmc_putc('.');
  dmc_putint((netif->ip_addr.addr & 0xff000000) >> 24);
  dmc_putc('\n');
  dmc_puts(TERMINAL_DEFAULT);

  // Jack 2019-03-28 debug
  for (uint16_t i = 0; i < 34; i++)
  {
    if ((i < 12) && (mark))
    {
      dmc_puts(TERMINAL_LIGHT_CYAN);
    }
    else
    {
      dmc_puts(TERMINAL_DEFAULT);
    }
    if (i == 12)
    {
      if (lwip_buf_1[12] == 0x08)
      {
        dmc_puts(TERMINAL_LIGHT_GREEN);
      }
      else
      {
        dmc_puts(TERMINAL_LIGHT_RED);
      }
    }
    // ARP / IP
    if (i == 13)
    {
      if (lwip_buf_1[13] == 0x00)
      {
        dmc_puts(TERMINAL_LIGHT_MAGENTA);
      }
      else if (lwip_buf_1[13] == 0x06)
      {
        dmc_puts(TERMINAL_LIGHT_CYAN);
      }
    }
    dmc_puthex2((u8_t *)lwip_buf_1[i]);
    dmc_putc(' ');
    if ((i == 12) || (i == 13))
    {
      dmc_puts(TERMINAL_DEFAULT);
    }

  }
  dmc_putc('\n');
#endif



//  for (uint16_t i = 0; i < sizeof(lwip_buf_1); i++)
//   {
//     pTXData[i + 5] = lwip_buf[i];
//   }

  if(tx_len != p->tot_len)
  {
    LWIP_PLATFORM_DIAG(("low_level_output_KSZ8851: error,tmplen=%"U32_F",tot_len=%"U32_F"\n\t", tx_len, p->tot_len));
    return ERR_BUF;
  }

  ksz8851snl_reset_tx_1();

  /*signal that packet should be sent();*/
  ksz8851_Send_1(lwip_buf_1, tx_len);

#if ETH_PAD_SIZE
  pbuf_header(p, ETH_PAD_SIZE); /* reclaim the padding word */
#endif
  
  LINK_STATS_INC(link.xmit);

  return ERR_OK;
}

/**
 * Should allocate a pbuf and transfer the bytes of the incoming
 * packet from the interface into the pbuf.
 *
 * @param netif the lwip network interface structure for this ethernetif
 * @return a pbuf filled with the received packet (including MAC header)
 *         NULL on memory error
 */
static struct pbuf * low_level_input_KSZ8851_1(struct netif *netif)
{
//  struct ethernetif *ethernetif = netif->state;
  struct pbuf *p, *q;
  u16_t len;
  u32_t rx_len = 0;

  // Jack 2019-03-27 Reads 0x0000!!!
//  uint16_t isr = ksz8851_ReadIntRegisterValue_1();
//  dmc_puthex4cr(isr);

//  if (ksz8851_IntHasOcurred_1())
//  {
////    dmc_puts("IntHasOcurred\n");
////      uint16_t isr = ksz8851_GetIntRegisterValue_1();
//
////    uint16_t isr = ksz8851_ReadIntRegisterValue_1();
//
////    dmc_puthex4cr(isr);
//  }
//  else
//  {
//    ksz8851snl_reset_tx_1();
//    ksz8851snl_reset_rx_1();
//    ksz8851_IntClearAll_1();
//    ksz8851_ResetIntHasOcurred_1();
//    return NULL;
//  }
  /* Obtain the size of the packet and put it into the "len"
     variable. */
  memset(lwip_buf_1, 0, MAX_FRAMELEN);

  len = ksz8851_Receive_1(lwip_buf_1, MAX_FRAMELEN);
//  dmc_puthex4cr(len);

//  ksz8851snl_reset_rx_1();
//  ksz8851_IntClearAll_1();

//  if (len <= 42)
  if (len == 0)
  {
    return NULL;
  }
  uint8_t *RxData = &lwip_buf_1[11];

//  if (RxData[12] == 0x00)
//  {
//    ksz8851snl_reset_rx_1();
//    return NULL;
//  }

#if (KSZ8851_DEBUG_IN_1)
//  dmc_puts(TERMINAL_YELLOW);
//  dmc_puts("low_level_input_E1 (M2)\n");
//  dmc_puts(TERMINAL_DEFAULT);
#endif

  ksz8851snl_reset_rx_1();

  uint8_t displaced = 0;
  if (RxData[12] != 0x08)
  {
    displaced = 1;
    if (RxData[8] == 0x08)
    {
      RxData = &lwip_buf_1[7];
    }
  }

#if (KSZ8851_DEBUG_IN_1)
  uint8_t MAC_ST[] = { 0x00, 0x80, 0xe1, 0x00, 0x00, 0x02 };
  uint8_t MAC_PC[] = { 0xb4, 0xb6, 0x86, 0x8e, 0x57, 0xbf };
  uint8_t mark = 0;
  if (memcmp(&RxData[0], MAC_ST, 6) == 0)
  {
    mark = 1;
  }
  if (memcmp(&RxData[6], MAC_ST, 6) == 0)
  {
    mark = 1;
  }
  if (memcmp(&RxData[0], MAC_PC, 6) == 0)
  {
    mark = 1;
  }
  if (memcmp(&RxData[6], MAC_PC, 6) == 0)
  {
    mark = 1;
  }

  // Jack 2019-03-28 debug
  if (mark)
  {
    dmc_puts(TERMINAL_YELLOW);
    dmc_puts("low_level_input_E1 (M2)\n");
    dmc_puts(TERMINAL_DEFAULT);
  for (uint16_t i = 0; i < 34; i++)
  {
    if ((i < 12) && (mark))
    {
      dmc_puts(TERMINAL_LIGHT_CYAN);
    }
    else
    {
      dmc_puts(TERMINAL_DEFAULT);
    }
    if (i == 12)
    {
      if (RxData[12] == 0x08)
      {
        dmc_puts(TERMINAL_LIGHT_GREEN);
      }
      else
      {
        dmc_puts(TERMINAL_LIGHT_RED);
      }
    }
    // ARP / IP
    if (i == 13)
    {
      if (RxData[13] == 0x00)
      {
        dmc_puts(TERMINAL_LIGHT_MAGENTA);
      }
      else if (RxData[13] == 0x06)
      {
        dmc_puts(TERMINAL_LIGHT_CYAN);
      }
    }
    dmc_puthex2(RxData[i]);
    dmc_putc(' ');
    if ((i == 12) || (i == 13))
    {
      dmc_puts(TERMINAL_DEFAULT);
    }
  }
  dmc_putc('\n');
  }
#endif


#if ETH_PAD_SIZE
  len += ETH_PAD_SIZE; /* allow room for Ethernet padding */
#endif

  /* We allocate a pbuf chain of pbufs from the pool. */
  p = pbuf_alloc(PBUF_RAW, len, PBUF_POOL);
  
  if (p != NULL) {

#if ETH_PAD_SIZE
    pbuf_header(p, -ETH_PAD_SIZE); /* drop the padding word */
#endif

    /* We iterate over the pbuf chain until we have read the entire
     * packet into the pbuf. */
    for(q = p; q != NULL; q = q->next) {
      /* Read enough bytes to fill this pbuf in the chain. The
       * available data in the pbuf is given by the q->len
       * variable.
       * This does not necessarily have to be a memcpy, you can also preallocate
       * pbufs for a DMA-enabled MAC and after receiving truncate it to the
       * actually received size. In this case, ensure the tot_len member of the
       * pbuf is the sum of the chained pbuf len members.
       */
      //read data into(q->payload, q->len);
      memcpy((u8_t *)q->payload, (u8_t *)&RxData[rx_len], q->len);
      rx_len += q->len;
    }
    // acknowledge that packet has been read();

    // When it is equal, it indicates that the data tail is reached.
    if( rx_len != p->tot_len )
    {
      return 0;
    }

//    // Jack 2019-03-28 debug
//    for (uint16_t i = 0; i < 32; i++)
//    {
////      dmc_puthex2((u8_t *)&q->payload[i]);
//      dmc_puthex2((u8_t *)(q->payload+i));
//      dmc_putc(' ');
//    }
//    dmc_putc('\n');


#if ETH_PAD_SIZE
    pbuf_header(p, ETH_PAD_SIZE); /* reclaim the padding word */
#endif

  }
//    LINK_STATS_INC(link.recv);
//  } else {
//    // drop packet();
//    LINK_STATS_INC(link.memerr);
//    LINK_STATS_INC(link.drop);
//  }

//  ksz8851snl_reset_tx_1();
//  ksz8851snl_reset_rx_1();
//  ksz8851_IntClearAll_1();
//  ksz8851_ResetIntHasOcurred_1();

  return p;  
}

/**
 * This function should be called when a packet is ready to be read
 * from the interface. It uses the function low_level_input() that
 * should handle the actual reception of bytes from the network
 * interface. Then the type of the received packet is determined and
 * the appropriate input function is called.
 *
 * @param netif the lwip network interface structure for this ethernetif
 */
void ethernetif_input_KSZ8851_1(struct netif *netif)
{
//  struct ethernetif_KSZ8851_1 *ethernetif;
  struct eth_hdr *ethhdr;
  struct pbuf *p;

//  ethernetif = netif->state;
//  dmc_putc(netif->name[0]);
//  dmc_putc(netif->name[1]);
//  dmc_putc('\n');

  /* move received packet into a new pbuf */
  p = low_level_input_KSZ8851_1(netif);
  /* no packet could be read, silently ignore this */
  if (p == NULL)
  {
    return;
  }
  /* points to packet payload, which starts with an Ethernet header */
  ethhdr = p->payload;

  switch (htons(ethhdr->type)) {
  /* IP or ARP packet? */
  case ETHTYPE_IP:
  case ETHTYPE_ARP:
#if PPPOE_SUPPORT
  /* PPPoE packet? */
  case ETHTYPE_PPPOEDISC:
  case ETHTYPE_PPPOE:
#endif /* PPPOE_SUPPORT */
    /* full packet send to tcpip_thread to process */
    if (netif->input(p, netif) != ERR_OK)
     {
       LWIP_DEBUGF(NETIF_DEBUG, ("ethernetif_input: IP input error\n"));
       pbuf_free(p);
       p = NULL;
     }
    break;

  default:
    pbuf_free(p);
    p = NULL;
    break;
  }
}

/**
 * Should be called at the beginning of the program to set up the
 * network interface. It calls the function low_level_init() to do the
 * actual setup of the hardware.
 *
 * This function should be passed as a parameter to netif_add().
 *
 * @param netif the lwip network interface structure for this ethernetif
 * @return ERR_OK if the loopif is initialized
 *         ERR_MEM if private data couldn't be allocated
 *         any other err_t on error
 */
err_t ethernetif_init_KSZ8851_1(struct netif *netif)
{
  struct ethernetif_KSZ8851_1 *ethernetif;

  LWIP_ASSERT("netif != NULL", (netif != NULL));
    
  ethernetif = mem_malloc(sizeof(struct ethernetif_KSZ8851_1));
  if (ethernetif == NULL) {
    LWIP_DEBUGF(NETIF_DEBUG, ("ethernetif_init: out of memory\n"));
    return ERR_MEM;
  }

#if LWIP_NETIF_HOSTNAME
  /* Initialize interface hostname */
  netif->hostname = "lwip";
#endif /* LWIP_NETIF_HOSTNAME */

  /*
   * Initialize the snmp variables and counters inside the struct netif.
   * The last argument should be replaced with your link speed, in units
   * of bits per second.
   */
//  NETIF_INIT_SNMP(netif, snmp_ifType_ethernet_csmacd, ???);

  netif->state = ethernetif;
  netif->name[0] = KSZ8851_IFNAME0_1;
  netif->name[1] = KSZ8851_IFNAME1_1;
  /* We directly use etharp_output() here to save a function call.
   * You can instead declare your own function an call etharp_output()
   * from it if you have to do some checks before sending (e.g. if link
   * is available...) */
  netif->output = etharp_output;
  netif->linkoutput = low_level_output_KSZ8851_1;

  ethernetif->ethaddr = (struct eth_addr *)&(netif->hwaddr[0]);

  /* initialize the hardware */
  low_level_init_KSZ8851_1(netif);
  
#if (KSZ8851_DEBUG_1)
  dmc_puts(TERMINAL_MAGENTA);
  dmc_puts("Init E1 M2 ");
  dmc_putc(netif->name[0]);
  dmc_putc(netif->name[1]);
  dmc_putc(' ');
  dmc_puthex2(netif->hwaddr[0]);
  dmc_putc(':');
  dmc_puthex2(netif->hwaddr[1]);
  dmc_putc(':');
  dmc_puthex2(netif->hwaddr[2]);
  dmc_putc(':');
  dmc_puthex2(netif->hwaddr[3]);
  dmc_putc(':');
  dmc_puthex2(netif->hwaddr[4]);
  dmc_putc(':');
  dmc_puthex2(netif->hwaddr[5]);
  dmc_putc(' ');
  dmc_putint(netif->ip_addr.addr & 0xff);
  dmc_putc('.');
  dmc_putint((netif->ip_addr.addr & 0xff00) >> 8);
  dmc_putc('.');
  dmc_putint((netif->ip_addr.addr & 0xff0000) >> 16);
  dmc_putc('.');
  dmc_putint((netif->ip_addr.addr & 0xff000000) >> 24);
  dmc_puts(TERMINAL_DEFAULT);
#endif

  return ERR_OK;
}

/**
  * @brief
  * @retval None
  */
void ethernet_link_check_state_KSZ8851_1(struct netif *netif)
{
  ETH_MACConfigTypeDef MACConf;
  uint32_t PHYLinkState;
  uint32_t linkchanged = 0, speed = 0, duplex =0;

//  dmc_puts("ethernet_link_check_state\n");

  PHYLinkState = KSZ8851_1_GetLinkState();

  if(netif_is_link_up(netif) && (PHYLinkState <= KSZ8851_1_STATUS_LINK_DOWN))
  {
//    HAL_ETH_Stop(&heth);
    netif_set_down(netif);
    netif_set_link_down(netif);
  }
  else if(!netif_is_link_up(netif) && (PHYLinkState > KSZ8851_1_STATUS_LINK_DOWN))
  {
    switch (PHYLinkState)
    {
    case KSZ8851_1_STATUS_100MBITS_FULLDUPLEX:
      duplex = ETH_FULLDUPLEX_MODE;
      speed = ETH_SPEED_100M;
      linkchanged = 1;
      break;
    case KSZ8851_1_STATUS_100MBITS_HALFDUPLEX:
      duplex = ETH_HALFDUPLEX_MODE;
      speed = ETH_SPEED_100M;
      linkchanged = 1;
      break;
    case KSZ8851_1_STATUS_10MBITS_FULLDUPLEX:
      duplex = ETH_FULLDUPLEX_MODE;
      speed = ETH_SPEED_10M;
      linkchanged = 1;
      break;
    case KSZ8851_1_STATUS_10MBITS_HALFDUPLEX:
      duplex = ETH_HALFDUPLEX_MODE;
      speed = ETH_SPEED_10M;
      linkchanged = 1;
      break;
    default:
      break;
    }

    if(linkchanged)
    {
      /* Get MAC Config MAC */
//      HAL_ETH_GetMACConfig(&heth, &MACConf);
      MACConf.DuplexMode = duplex;
      MACConf.Speed = speed;
//      HAL_ETH_SetMACConfig(&heth, &MACConf);
//      HAL_ETH_Start(&heth);
      netif_set_up(netif);
      netif_set_link_up(netif);
    }
  }
}

//#endif /* 0 */
