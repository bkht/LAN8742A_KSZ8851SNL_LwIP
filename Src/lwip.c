/**
 ******************************************************************************
  * File Name          : LWIP.c
  * Description        : This file provides initialization code for LWIP
  *                      middleWare.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
  
/* Includes ------------------------------------------------------------------*/
#include "lwip.h"
#include "lwip/init.h"
#include "lwip/netif.h"
#if defined ( __CC_ARM )  /* MDK ARM Compiler */
#include "lwip/sio.h"
#endif /* MDK ARM Compiler */

#define LAN8742           1
#define KSZ8851_0         1
#define KSZ8851_1         1

#if (KSZ8851_0)
#include "ethernetif_ksz8851_0.h"
#endif
#if (KSZ8851_1)
#include "ethernetif_ksz8851_1.h"
#endif

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */
/* Private function prototypes -----------------------------------------------*/
/* ETH Variables initialization ----------------------------------------------*/
void Error_Handler(void);

/* DHCP Variables initialization ---------------------------------------------*/
//uint32_t DHCPfineTimer = 0;
//uint32_t DHCPcoarseTimer = 0;
/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/* Variables Initialization */
#if (LAN8742)
struct netif gnetif0;
ip4_addr_t ipaddr0;
ip4_addr_t netmask0;
ip4_addr_t gw0;
uint8_t IP_ADDRESS0[4];
#endif

#if (KSZ8851_0)
struct netif gnetif1;
ip4_addr_t ipaddr1;
ip4_addr_t netmask1;
ip4_addr_t gw1;
uint8_t IP_ADDRESS1[4];
#endif

#if (KSZ8851_1)
struct netif gnetif2;
ip4_addr_t ipaddr2;
ip4_addr_t netmask2;
ip4_addr_t gw2;
uint8_t IP_ADDRESS2[4];
#endif

#if (LAN8742 | KSZ8851_0 | KSZ8851_1)
uint8_t NETMASK_ADDRESS[4];
uint8_t GATEWAY_ADDRESS[4];
#endif

/* USER CODE BEGIN 2 */

/* USER CODE END 2 */

/**
  * LwIP initialization function
  */
void MX_LWIP_Init(void)
{
  /* IP addresses initialization */
  // https://github.com/yonch/lwip-example/blob/master/mch_main.c
#if (LAN8742)
  IP_ADDRESS0[0]     = 192;
  IP_ADDRESS0[1]     = 168;
  IP_ADDRESS0[2]     = 25;
  IP_ADDRESS0[3]     = 232;
#endif

#if (KSZ8851_0)
  IP_ADDRESS1[0]     = 192;
  IP_ADDRESS1[1]     = 168;
  IP_ADDRESS1[2]     = 25;
  IP_ADDRESS1[3]     = 233;
#endif

#if (KSZ8851_1)
  IP_ADDRESS2[0]     = 192;
  IP_ADDRESS2[1]     = 168;
  IP_ADDRESS2[2]     = 25;
  IP_ADDRESS2[3]     = 234;
#endif

#if (LAN8742 | KSZ8851_0 | KSZ8851_1)
  NETMASK_ADDRESS[0] = 255;
  NETMASK_ADDRESS[1] = 255;
  NETMASK_ADDRESS[2] = 255;
  NETMASK_ADDRESS[3] = 0;

  GATEWAY_ADDRESS[0] = 192;
  GATEWAY_ADDRESS[1] = 168;
  GATEWAY_ADDRESS[2] = 25;
  GATEWAY_ADDRESS[3] = 253;
#endif

  /* Initilialize the LwIP stack without RTOS */
  lwip_init();

#if (LAN8742)
  /* IP addresses initialization without DHCP (IPv4) */
  IP4_ADDR(&ipaddr0, IP_ADDRESS0[0], IP_ADDRESS0[1], IP_ADDRESS0[2], IP_ADDRESS0[3]);
  IP4_ADDR(&netmask0, NETMASK_ADDRESS[0], NETMASK_ADDRESS[1] , NETMASK_ADDRESS[2], NETMASK_ADDRESS[3]);
  IP4_ADDR(&gw0, GATEWAY_ADDRESS[0], GATEWAY_ADDRESS[1], GATEWAY_ADDRESS[2], GATEWAY_ADDRESS[3]);

  /* add the network interface (IPv4/IPv6) without RTOS */
  netif_add(&gnetif0, &ipaddr0, &netmask0, &gw0, NULL, &ethernetif_init, &ethernet_input);
#endif

#if (KSZ8851_0)
  /* IP addresses initialization without DHCP (IPv4) */
  IP4_ADDR(&ipaddr1, IP_ADDRESS1[0], IP_ADDRESS1[1], IP_ADDRESS1[2], IP_ADDRESS1[3]);
  IP4_ADDR(&netmask1, NETMASK_ADDRESS[0], NETMASK_ADDRESS[1] , NETMASK_ADDRESS[2], NETMASK_ADDRESS[3]);
  IP4_ADDR(&gw1, GATEWAY_ADDRESS[0], GATEWAY_ADDRESS[1], GATEWAY_ADDRESS[2], GATEWAY_ADDRESS[3]);

  /* add the network interface (IPv4/IPv6) without RTOS */
  netif_add(&gnetif1, &ipaddr1, &netmask1, &gw1, NULL, &ethernetif_init_KSZ8851_0, &ethernet_input);
#endif

#if (KSZ8851_1)
  /* IP addresses initialization without DHCP (IPv4) */
  IP4_ADDR(&ipaddr2, IP_ADDRESS2[0], IP_ADDRESS2[1], IP_ADDRESS2[2], IP_ADDRESS2[3]);
  IP4_ADDR(&netmask2, NETMASK_ADDRESS[0], NETMASK_ADDRESS[1] , NETMASK_ADDRESS[2], NETMASK_ADDRESS[3]);
  IP4_ADDR(&gw2, GATEWAY_ADDRESS[0], GATEWAY_ADDRESS[1], GATEWAY_ADDRESS[2], GATEWAY_ADDRESS[3]);

  /* add the network interface (IPv4/IPv6) without RTOS */
  netif_add(&gnetif2, &ipaddr2, &netmask2, &gw2, NULL, &ethernetif_init_KSZ8851_1, &ethernet_input);
#endif

  /* Registers the default network interface */
#if (LAN8742)
  netif_set_default(&gnetif0);
#elif  (KSZ8851_0)
  netif_set_default(&gnetif1);
#elif  (KSZ8851_1)
  netif_set_default(&gnetif2);
#endif

#if (LAN8742)
  if (netif_is_link_up(&gnetif0))
  {
    /* When the netif is fully configured this function must be called */
    netif_set_up(&gnetif0);
  }
  else
  {
    /* When the netif link is down this function must be called */
    netif_set_down(&gnetif0);
  }
#endif

#if (KSZ8851_0)
  if (netif_is_link_up(&gnetif1))
  {
    /* When the netif is fully configured this function must be called */
    netif_set_up(&gnetif1);
  }
  else
  {
    /* When the netif link is down this function must be called */
    netif_set_down(&gnetif1);
  }
#endif

#if (KSZ8851_1)
  if (netif_is_link_up(&gnetif2))
  {
    /* When the netif is fully configured this function must be called */
    netif_set_up(&gnetif2);
  }
  else
  {
    /* When the netif link is down this function must be called */
    netif_set_down(&gnetif2);
  }
#endif

/* USER CODE BEGIN 3 */

/* USER CODE END 3 */
}

#ifdef USE_OBSOLETE_USER_CODE_SECTION_4
/* Kept to help code migration. (See new 4_1, 4_2... sections) */
/* Avoid to use this user section which will become obsolete. */
/* USER CODE BEGIN 4 */
/* USER CODE END 4 */
#endif

/**
 * ----------------------------------------------------------------------
 * Function given to help user to continue LwIP Initialization
 * Up to user to complete or change this function ...
 * Up to user to call this function in main.c in while (1) of main(void) 
 *-----------------------------------------------------------------------
 * Read a received packet from the Ethernet buffers 
 * Send it to the lwIP stack for handling
 * Handle timeouts if LWIP_TIMERS is set and without RTOS
 * Handle the llink status if LWIP_NETIF_LINK_CALLBACK is set and without RTOS 
 */
void MX_LWIP_Process(void)
{
/* USER CODE BEGIN 4_1 */
/* USER CODE END 4_1 */
#if (LAN8742)
  ethernetif_input(&gnetif0);
#endif

#if (KSZ8851_0)
  ethernetif_input_KSZ8851_0(&gnetif1);
#endif

#if (KSZ8851_1)
  ethernetif_input_KSZ8851_1(&gnetif2);
#endif

/* USER CODE BEGIN 4_2 */
/* USER CODE END 4_2 */  
  /* Handle timeouts */
  sys_check_timeouts();

/* USER CODE BEGIN 4_3 */
/* USER CODE END 4_3 */
}

#if defined ( __CC_ARM )  /* MDK ARM Compiler */
/**
 * Opens a serial device for communication.
 *
 * @param devnum device number
 * @return handle to serial device if successful, NULL otherwise
 */
sio_fd_t sio_open(u8_t devnum)
{
  sio_fd_t sd;

/* USER CODE BEGIN 7 */
  sd = 0; // dummy code
/* USER CODE END 7 */
	
  return sd;
}

/**
 * Sends a single character to the serial device.
 *
 * @param c character to send
 * @param fd serial device handle
 *
 * @note This function will block until the character can be sent.
 */
void sio_send(u8_t c, sio_fd_t fd)
{
/* USER CODE BEGIN 8 */
/* USER CODE END 8 */
}

/**
 * Reads from the serial device.
 *
 * @param fd serial device handle
 * @param data pointer to data buffer for receiving
 * @param len maximum length (in bytes) of data to receive
 * @return number of bytes actually received - may be 0 if aborted by sio_read_abort
 *
 * @note This function will block until data can be received. The blocking
 * can be cancelled by calling sio_read_abort().
 */
u32_t sio_read(sio_fd_t fd, u8_t *data, u32_t len)
{
  u32_t recved_bytes;

/* USER CODE BEGIN 9 */
  recved_bytes = 0; // dummy code
/* USER CODE END 9 */	
  return recved_bytes;
}

/**
 * Tries to read from the serial device. Same as sio_read but returns
 * immediately if no data is available and never blocks.
 *
 * @param fd serial device handle
 * @param data pointer to data buffer for receiving
 * @param len maximum length (in bytes) of data to receive
 * @return number of bytes actually received
 */
u32_t sio_tryread(sio_fd_t fd, u8_t *data, u32_t len)
{
  u32_t recved_bytes;

/* USER CODE BEGIN 10 */
  recved_bytes = 0; // dummy code
/* USER CODE END 10 */	
  return recved_bytes;
}
#endif /* MDK ARM Compiler */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
