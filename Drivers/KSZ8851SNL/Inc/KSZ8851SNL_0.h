/*
 * The MIT License (MIT)
 *
 * Copyright (c) [2018] [Marco Russi]
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#ifndef __KSZ8851SNL_0_H
#define __KSZ8851SNL_0_H

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include "stm32h7xx_hal.h"
#include "main.h"
// #include "cmsis_os.h"
#include "conf_eth.h"

#ifdef __cplusplus
extern "C"
{
#endif

#define INT_SPI_CALLBACK						HAL_SPI_TxRxCpltCallback
#define SPI_HANDLE_TYPE							SPI_HandleTypeDef
#define INT_EXT_GPIO_CALLBACK					EXTI1_IRQHandler

// max frame length which the conroller will accept:
#define   MAX_FRAMELEN    1518        // (note: maximum ethernet frame length would be 1518)

#define KSZ8851_0_STATUS_READ_ERROR            ((int32_t)-5)
#define KSZ8851_0_STATUS_WRITE_ERROR           ((int32_t)-4)
#define KSZ8851_0_STATUS_ADDRESS_ERROR         ((int32_t)-3)
#define KSZ8851_0_STATUS_RESET_TIMEOUT         ((int32_t)-2)
#define KSZ8851_0_STATUS_ERROR                 ((int32_t)-1)
#define KSZ8851_0_STATUS_OK                    ((int32_t) 0)
#define KSZ8851_0_STATUS_LINK_DOWN             ((int32_t) 1)
#define KSZ8851_0_STATUS_100MBITS_FULLDUPLEX   ((int32_t) 2)
#define KSZ8851_0_STATUS_100MBITS_HALFDUPLEX   ((int32_t) 3)
#define KSZ8851_0_STATUS_10MBITS_FULLDUPLEX    ((int32_t) 4)
#define KSZ8851_0_STATUS_10MBITS_HALFDUPLEX    ((int32_t) 5)
#define KSZ8851_0_STATUS_AUTONEGO_NOTDONE      ((int32_t) 6)


typedef enum
{
	INT_SPI_READY_0,
	INT_SPI_BUSY_0,
	INT_SPI_ERROR_0
} spi_int_codes;

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

#define CLEAR_GPIO_INT_FLAG()		__HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_1)

uint8_t ksz8851_get_spi_state_0(SPI_HandleTypeDef *);
void ksz8851_set_pm_0(uint8_t mode);
void ksz8851_send_packet_0(uint8_t *buf, uint16_t length);
uint16_t ksz8851_read_packet_0(uint8_t *buf, uint16_t limit);
uint8_t ksz8851_has_data_0(void);
void ksz8851_irq_0(void);
void ksz8851_clr_irq_0(void);

uint16_t ksz8851_reg_read_0(uint16_t);
void ksz8851_reg_write_0(uint16_t, uint16_t);
void ksz8851_reg_setbits_0(uint16_t, uint16_t);
void ksz8851_reg_clrbits_0(uint16_t, uint16_t);
void ksz8851_fifo_read_0(uint8_t *, uint16_t);
void ksz8851_fifo_write_0(uint8_t *, uint16_t);
uint8_t ksz8851_init_0(void);

uint8_t ksz8851_interface_init_0(void);
void ksz8851_hard_reset_0(void);
void ksz8851_soft_reset_0(uint8_t queue_only);
void ksz8851_set_registers_0(void);
uint32_t ksz8851_reinit_0(void);
uint32_t ksz8851_MIBCountersRead_0(uint16_t offset);
void KSZ8851SNL_ReadMIBCounters_0(char* param);
void ksz8851_AllRegistersDump_0(void);
void ksz8851_RegistersDump_0(void);
void ksz8851_IntEnable_0(void);
void ksz8851_IntDisable_0(void);
void ksz8851_IntClearAll_0(void);
void ksz8851_IntClearFlags_0(uint16_t flags);
uint16_t ksz8851_IntGet_0(void);
void ksz8851_PMECRStatusClear_0(uint16_t flags);
uint16_t ksz8851_RXQCRGet_0(void);
uint16_t ksz8851_FrameCounterSet(void);
void ksz8851_TxQueueReset_0(void);
void ksz8851_RxQueueReset_0(void);
uint32_t ksz8851snl_reset_rx_0(void);
uint32_t ksz8851snl_reset_tx_0(void);
uint16_t ksz8851_FrameCounterGet_0(void);
void ksz8851_Enable_0(void);
bool ksz8851_TransmitBegin_0(uint16_t length);
void ksz8851_Transmit_0(uint16_t length, uint8_t *buffer);
void ksz8851_TransmitEnd_0(uint16_t length);
void ksz8851_ReleaseIncosistentFrame_0(void);
void ksz8851_Send_0(uint8_t *pTXData, uint16_t pTXLength);
uint16_t ksz8851_Receive_0(uint8_t *pRXData, uint16_t pRXLength);
void ksz8851_GetMacAddress_0(uint8_t *macAddress);
uint16_t ksz8851_PHYStatusGet_0(void);
void ksz8851_SetDigitalLoopbackMode_0(void);
void ksz8851_EnableInterrupts_0(void);
uint16_t ksz8851_CheckIrqStat_0(void);
uint16_t ksz8851_CurrFrameSize_0(void);
uint8_t ksz8851_DwordAllignDiff_0(uint8_t val);
uint32_t ksz8851_CalcCrc_0(const void *data, size_t length);
void ksz8851_intrn_0(void);
void ksz8851_irq_0(void);
void ksz8851_clr_irq_0(void);
void ksz8851_ClearRxInterrupt_0(void);
void ksz8851_EnableRxInterrupt_0(void);
void ksz8851_DisableRxInterrupt_0(void);
uint16_t ksz8851_ReadRxFrameCount_0(void);
uint16_t ksz8851_ReadRxByteCount_0(void);
uint16_t ksz8851_ReadRxHeaderStatus_0(void);
void ksz8851_ClearRxFramePointer_0(void);
void ksz8851_SetRxFramePointerAutoIncrement_0(void);
void ksz8851_ClrRxFramePointerAutoIncrement_0(void);
void ksz8851_EnableRXQReadAccess_0(void);
void ksz8851_DisableRXQReadAccess_0(void);
uint16_t ksz8851_ReadRxInterruptSource_0(void);
uint16_t ksz8851_read_id_0(void);
void ksz8851_ResetIntHasOcurred_0(void);
uint8_t ksz8851_IntHasOcurred_0(void);
uint16_t ksz8851_GetIntRegisterValue_0(void);
uint16_t ksz8851_ReadIntRegisterValue_0(void);
void ksz8851_IntHandler_0();
void ksz8851_show_isr_0(uint16_t isr_reg_0);
bool ksz8851_CheckQMUTXQHasSpace_0(uint16_t length);

void clr_dma_tx_ended_0(void);
void set_dma_tx_ended_0(void);
void wait_dma_tx_ended_0(void);
void clr_dma_rx_ended_0(void);
void set_dma_rx_ended_0(void);
void wait_dma_rx_ended_0(void);

uint32_t KSZ8851_0_GetLinkState(void);

#ifdef __cplusplus
}
#endif

#endif
