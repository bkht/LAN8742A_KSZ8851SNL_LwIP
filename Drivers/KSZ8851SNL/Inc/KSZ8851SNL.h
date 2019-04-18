#ifndef __KSZ8851SNL_H
#define __KSZ8851SNL_H

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include "stm32h7xx_hal.h"
#include "main.h"
#include <spi.h>
//#include "conf_eth.h"
#include "ksz8851snl_reg.h"

#ifdef __cplusplus
extern "C"
{
#endif

#define KSZ8851_USE_TX_DMA        0
#define KSZ8851_USE_RX_DMA        0
#define KSZ8851_USE_HARD_RESET    0

#define INT_SPI_CALLBACK						HAL_SPI_TxRxCpltCallback
#define SPI_HANDLE_TYPE							SPI_HandleTypeDef
#define INT_EXT_GPIO_CALLBACK					EXTI1_IRQHandler

// max frame length which the conroller will accept:
#define   MAX_FRAMELEN    1518        // (note: maximum ethernet frame length would be 1518)

/** Maximum transfer unit. */
#define NET_MTU								    1500

/** Network link speed. */
#define NET_LINK_SPEED						100000000

#define MAX_FRAMES_IN_RXQ	        16

#define KSZ8851_STATUS_READ_ERROR            ((int32_t)-5)
#define KSZ8851_STATUS_WRITE_ERROR           ((int32_t)-4)
#define KSZ8851_STATUS_ADDRESS_ERROR         ((int32_t)-3)
#define KSZ8851_STATUS_RESET_TIMEOUT         ((int32_t)-2)
#define KSZ8851_STATUS_ERROR                 ((int32_t)-1)
#define KSZ8851_STATUS_OK                    ((int32_t) 0)
#define KSZ8851_STATUS_LINK_DOWN             ((int32_t) 1)
#define KSZ8851_STATUS_100MBITS_FULLDUPLEX   ((int32_t) 2)
#define KSZ8851_STATUS_100MBITS_HALFDUPLEX   ((int32_t) 3)
#define KSZ8851_STATUS_10MBITS_FULLDUPLEX    ((int32_t) 4)
#define KSZ8851_STATUS_10MBITS_HALFDUPLEX    ((int32_t) 5)
#define KSZ8851_STATUS_AUTONEGO_NOTDONE      ((int32_t) 6)

#define KSZ8851_LINK_STATE_DOWN              0
#define KSZ8851_LINK_STATE_UP                1
#define KSZ8851_LINK_SPEED_10MB              0
#define KSZ8851_LINK_SPEED_100MB             1
#define KSZ8851_LINK_DUPLEX_HALF             0
#define KSZ8851_LINK_DUPLEX_FULL             1

struct KSZ8851_LINK_STATE
{
  uint8_t state;
  uint8_t link;
  uint8_t speed;
  uint8_t duplex;
};

typedef enum
{
	INT_SPI_STATE_READY,
	INT_SPI_STATE_BUSY,
	INT_SPI_STATE_ERROR
} int_spi_state_codes;

struct KSZ8851_INTERFACE
{
  uint32_t cs_port;
  uint16_t cs_pin;
  uint32_t rst_port;
  uint16_t rst_pin;
  SPI_HandleTypeDef *hspi;
  uint8_t MAC_address[6];

  uint16_t rxqcr;
  uint16_t frameId;
  uint8_t rxFrameCount;
  uint8_t dma_rx_ended;
  uint8_t dma_tx_ended;
  uint16_t isr_old;
  uint8_t isr_ocurred;
  uint16_t isr_reg;
  uint8_t link;
  uint8_t speed;
  uint8_t duplex;
  uint8_t state;
};

//#define SET_SPI_CS_PIN_NO_DELAY()	{ \
//										HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_SET); \
//									}
//
//#define SET_SPI_CS_PIN()			{ \
//										HAL_Delay(2); \
//										HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_SET); \
//									}
//
//#define RESET_SPI_CS_PIN()			{ \
//										HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_RESET); \
//										HAL_Delay(2); \
//									}

//#define CLEAR_GPIO_INT_FLAG()		__HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_1)

void ksz8851_init(struct KSZ8851_INTERFACE *interface, SPI_HandleTypeDef *hspi,
    uint32_t cs_port, uint16_t cs_pin, uint32_t rst_port, uint16_t rst_pin, uint8_t *MACaddress);
uint8_t ksz8851_interface_init(struct KSZ8851_INTERFACE *interface);
uint8_t ksz8851_init_chip(struct KSZ8851_INTERFACE *interface);

void ksz8851_hard_reset(struct KSZ8851_INTERFACE *interface);
void ksz8851_soft_reset(struct KSZ8851_INTERFACE *interface, uint8_t queue_only);
uint16_t ksz8851_reg_read(struct KSZ8851_INTERFACE *interface, uint16_t reg);
void ksz8851_reg_write(struct KSZ8851_INTERFACE *interface, uint16_t reg, uint16_t wrdata);
void ksz8851_reg_setbits(struct KSZ8851_INTERFACE *interface, uint16_t reg, uint16_t bits_to_set);
void ksz8851_reg_clrbits(struct KSZ8851_INTERFACE *interface, uint16_t reg, uint16_t bits_to_clr);

void ksz8851_IntEnable(struct KSZ8851_INTERFACE *interface);
void ksz8851_IntDisable(struct KSZ8851_INTERFACE *interface);
void ksz8851_IntClearAll(struct KSZ8851_INTERFACE *interface);
void ksz8851_IntClearFlags(struct KSZ8851_INTERFACE *interface, uint16_t flags);
uint16_t ksz8851_read_packet(struct KSZ8851_INTERFACE *interface, uint8_t *buf, uint16_t limit);
uint8_t ksz8851_has_data(struct KSZ8851_INTERFACE *interface);
uint8_t ksz8851_get_spi_state(struct KSZ8851_INTERFACE *interface);
uint16_t ksz8851_fifo_read(struct KSZ8851_INTERFACE *interface, uint8_t *buf, uint16_t len);
void ksz8851_fifo_write(struct KSZ8851_INTERFACE *interface, uint8_t *buf, uint16_t len);
uint32_t ksz8851_MIBCountersRead(struct KSZ8851_INTERFACE *interface, uint16_t offset);
void KSZ8851SNL_ReadMIBCounters(struct KSZ8851_INTERFACE *interface, char* param);
void ksz8851_AllRegistersDump(struct KSZ8851_INTERFACE *interface);
void ksz8851_RegistersDump(struct KSZ8851_INTERFACE *interface);

uint16_t ksz8851_IntGet(struct KSZ8851_INTERFACE *interface);
void ksz8851_PMECRStatusClear(struct KSZ8851_INTERFACE *interface, uint16_t flags);
uint16_t ksz8851_RXQCRGet(struct KSZ8851_INTERFACE *interface);
uint16_t ksz8851_FrameCounterSet(struct KSZ8851_INTERFACE *interface);
void ksz8851_TxQueueReset(struct KSZ8851_INTERFACE *interface);
void ksz8851_RxQueueReset(struct KSZ8851_INTERFACE *interface);

uint32_t ksz8851snl_reset_rx(struct KSZ8851_INTERFACE *interface);
uint32_t ksz8851snl_reset_tx(struct KSZ8851_INTERFACE *interface);

void ksz8851_Enable(struct KSZ8851_INTERFACE *interface);
bool ksz8851_TransmitBegin(struct KSZ8851_INTERFACE *interface, uint16_t length);
void ksz8851_Transmit(struct KSZ8851_INTERFACE *interface, uint16_t length, uint8_t *buffer);
void ksz8851_TransmitEnd(struct KSZ8851_INTERFACE *interface, uint16_t length);
void ksz8851_ReleaseIncosistentFrame(struct KSZ8851_INTERFACE *interface);

void ksz8851_Send(struct KSZ8851_INTERFACE *interface, uint8_t *pTXData, uint16_t pTXLength);
uint16_t ksz8851_Receive(struct KSZ8851_INTERFACE *interface, uint8_t *pRXData, uint16_t pRXLength);

uint16_t ksz8851_PHYStatusGet(struct KSZ8851_INTERFACE *interface);
void ksz8851_SetDigitalLoopbackMode(struct KSZ8851_INTERFACE *interface);
void ksz8851_EnableInterrupts(struct KSZ8851_INTERFACE *interface);

uint16_t ksz8851_CheckIrqStat(struct KSZ8851_INTERFACE *interface);
uint16_t ksz8851_CurrFrameSize(struct KSZ8851_INTERFACE *interface);
uint8_t ksz8851_DwordAllignDiff(uint8_t val);
uint32_t ksz8851_CalcCrc(const void *data, size_t length);

void ksz8851_irq(struct KSZ8851_INTERFACE *interface);
void ksz8851_clr_irq(struct KSZ8851_INTERFACE *interface);
void ksz8851_ClearRxInterrupt(struct KSZ8851_INTERFACE *interface);
void ksz8851_EnableRxInterrupt(struct KSZ8851_INTERFACE *interface);
void ksz8851_DisableRxInterrupt(struct KSZ8851_INTERFACE *interface);
uint16_t ksz8851_ReadRxFrameCount(struct KSZ8851_INTERFACE *interface);
uint16_t ksz8851_ReadRxByteCount(struct KSZ8851_INTERFACE *interface);
uint16_t ksz8851_ReadRxHeaderStatus(struct KSZ8851_INTERFACE *interface);
void ksz8851_ClearRxFramePointer(struct KSZ8851_INTERFACE *interface);
void ksz8851_SetRxFramePointerAutoIncrement(struct KSZ8851_INTERFACE *interface);
void ksz8851_ClrRxFramePointerAutoIncrement(struct KSZ8851_INTERFACE *interface);
void ksz8851_EnableRXQReadAccess(struct KSZ8851_INTERFACE *interface);
void ksz8851_DisableRXQReadAccess(struct KSZ8851_INTERFACE *interface);
uint16_t ksz8851_ReadRxInterruptSource(struct KSZ8851_INTERFACE *interface);
uint16_t ksz8851_read_id(struct KSZ8851_INTERFACE *interface);
uint16_t ksz8851_ReadIntRegisterValue(struct KSZ8851_INTERFACE *interface);
void ksz8851_IntHandler(struct KSZ8851_INTERFACE *interface);
uint8_t ksz8851_has_isr_RXI(uint16_t isr_reg);
void ksz8851_show_isr(struct KSZ8851_INTERFACE *interface, uint16_t isr_reg);
bool ksz8851_CheckQMUTXQHasSpace(struct KSZ8851_INTERFACE *interface, uint16_t length);
void clr_dma_tx_ended(struct KSZ8851_INTERFACE *interface);
void set_dma_tx_ended(struct KSZ8851_INTERFACE *interface);
void wait_dma_tx_ended(struct KSZ8851_INTERFACE *interface);
void clr_dma_rx_ended(struct KSZ8851_INTERFACE *interface);
void set_dma_rx_ended(struct KSZ8851_INTERFACE *interface);
void wait_dma_rx_ended(struct KSZ8851_INTERFACE *interface);
uint32_t KSZ8851_GetLinkState(struct KSZ8851_INTERFACE *interface);

#ifdef __cplusplus
}
#endif

#endif
