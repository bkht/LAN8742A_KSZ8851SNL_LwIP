#ifndef __KSZ8851SNL_1_H
#define __KSZ8851SNL_1_H

#include "stm32h7xx_hal.h"
#include "main.h"
#include "ksz8851snl.h"

#ifdef __cplusplus
extern "C"
{
#endif

struct KSZ8851_INTERFACE KSZ8851_interface_1;

uint8_t ksz8851_init_1(void);
void ksz8851_IntEnable_1(void);
void ksz8851snl_reset_tx_1(void);
void ksz8851snl_reset_rx_1(void);
void ksz8851_IntClearAll_1(void);
void ksz8851_Send_1(uint8_t *pTXData, uint16_t pTXLength);
uint16_t ksz8851_Receive_1(uint8_t *pRXData, uint16_t pRXLength);
uint32_t KSZ8851_1_GetLinkState();

#ifdef __cplusplus
}
#endif

#endif
