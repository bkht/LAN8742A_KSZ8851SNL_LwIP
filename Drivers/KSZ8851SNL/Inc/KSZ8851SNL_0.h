#ifndef __KSZ8851SNL_0_H
#define __KSZ8851SNL_0_H

#include "stm32h7xx_hal.h"
#include "main.h"
#include "ksz8851snl.h"

#ifdef __cplusplus
extern "C"
{
#endif

struct KSZ8851_INTERFACE KSZ8851_interface_0;

uint8_t ksz8851_init_0(void);
void ksz8851_IntEnable_0(void);
void ksz8851snl_reset_tx_0(void);
void ksz8851snl_reset_rx_0(void);
void ksz8851_IntClearAll_0(void);
void ksz8851_Send_0(uint8_t *pTXData, uint16_t pTXLength);
uint16_t ksz8851_Receive_0(uint8_t *pRXData, uint16_t pRXLength);
uint32_t KSZ8851_0_GetLinkState();

#ifdef __cplusplus
}
#endif

#endif
