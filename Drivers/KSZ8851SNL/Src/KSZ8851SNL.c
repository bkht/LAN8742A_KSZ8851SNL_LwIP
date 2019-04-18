/*
 * The MIT License (MIT)
 *
 *
 * https://github.com/fabiobaltieri/avr-nrf/blob/master/firmware/ksz8851snl.c
 * https://github.com/marcorussi/iot_eth_server
 * https://siliconlabs.github.io/Gecko_SDK_Doc/efr32mg1/html/ksz8851snl_8c_source.html
 * https://github.com/avrxml/asf/blob/master/sam/components/ethernet_phy/ksz8851snl/ksz8851snl.c
 * https://www.oryx-embedded.com/doc/ksz8851_8c_source.html
 * https://code.zoetrope.io/bfo/nrf52-freertos/blob/edf7bd8206281b3e5d9c71e39c9c8ed2b0710ce6/lib/FreeRTOS-Plus-TCP/portable/NetworkInterface/ksz8851snl/ksz8851snl.c
 * TODO Check below code
 * https://github.com/atx/avr-uip-2/blob/master/drivers/ksz8851/ksz8851.c
 * https://github.com/Velleman/VM204-Firmware/blob/master/cyclone_tcp/drivers/ksz8851.c
 * https://github.com/RevolutionPi/piControl/blob/master/ksz8851.c
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

/* ------------------- Local inclusions -------------------- */
#include <app_ethernet.h>
#include <dmc_print.h>
#include <dmc_tcp.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <gpio.h>
#include <dmc_terminal.h>
#include "stm32h7xx_hal.h"
#include "ksz8851snl.h"

/* ------------------- Local variables -------------------- */

/* ------------------- Functions -------------------------- */

void ksz8851_init(struct KSZ8851_INTERFACE *interface, SPI_HandleTypeDef *hspi,
    uint32_t cs_port, uint16_t cs_pin, uint32_t rst_port, uint16_t rst_pin, uint8_t *MACaddress)
{
  interface->hspi = hspi;
  interface->cs_port = cs_port;
  interface->cs_pin = cs_pin;
  interface->rst_port = rst_port;
  interface->rst_pin = rst_pin;
  memcpy(interface->MAC_address, MACaddress, 6);

  ksz8851_init_chip(interface);
}

/**
 * @brief Initialize SPI bus
 * @param None
 * @return Error code
 */
uint8_t ksz8851_interface_init(struct KSZ8851_INTERFACE *interface)
{
  dmc_puts("ksz8851_interface_init\n");

  /* Configure the SPI peripheral */
  /* Set the SPI parameters */
//  if (interface->hspi->Instance == SPI1)
//  {
//    dmc_puts("SPI1...\n");
//  }

//#if (KSZ8851_USE_SPI == 1)
//  interface.hspi->Instance = SPI1;
//#endif
//#if (KSZ8851_USE_SPI == 2)
//  SpiHandle.Instance = SPI2;
//#endif
//#if (KSZ8851_USE_SPI == 3)
//  SpiHandle.Instance = SPI3;
//#endif
//#if (KSZ8851_USE_SPI == 4)
//  SpiHandle.Instance = SPI4;
//#endif
//#if (KSZ8851_USE_SPI == 5)
//  SpiHandle.Instance = SPI5;
//#endif
  interface->hspi->Init.Mode = SPI_MODE_MASTER;
  interface->hspi->Init.Direction = SPI_DIRECTION_2LINES;
  interface->hspi->Init.DataSize = SPI_DATASIZE_8BIT;
  interface->hspi->Init.CLKPolarity = SPI_POLARITY_LOW;
  interface->hspi->Init.CLKPhase = SPI_PHASE_1EDGE;
  interface->hspi->Init.NSS = SPI_NSS_SOFT;  // SPI_NSS_HARD_OUTPUT
  // SPI1, 2, 3 100MHz
  // SPI4, 5    100MHz
  // SPI4: 100MHz /  8 = 12.5 MHz: SPI_BAUDRATEPRESCALER_8
  interface->hspi->Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;  // KSZ8851SNL can handle up to 50 MHz. 256: 0.78125
  interface->hspi->Init.FirstBit = SPI_FIRSTBIT_MSB;
  interface->hspi->Init.TIMode = SPI_TIMODE_DISABLE;
  interface->hspi->Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  interface->hspi->Init.CRCPolynomial = 7;
  interface->hspi->Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  interface->hspi->Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  interface->hspi->Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  interface->hspi->Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  interface->hspi->Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  interface->hspi->Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  interface->hspi->Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  interface->hspi->Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  interface->hspi->Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  interface->hspi->Init.IOSwap = SPI_IO_SWAP_DISABLE;
  if (HAL_SPI_Init(interface->hspi) != HAL_OK)
  {
    return false;
  }

  /* Pull the CS pin low */
  HAL_GPIO_WritePin((GPIO_TypeDef*) interface->cs_port, interface->cs_pin, GPIO_PIN_RESET);

  return true;
}

/**
 * \brief KSZ8851 controller initialization
 * \param None
 * \return Error code
 */
uint8_t ksz8851_init_chip(struct KSZ8851_INTERFACE *interface)
{
  uint16_t dev_id = 0;
//  uint8_t MAC_address[] = { 0x00, 0x80, 0xe1, 0x00, 0x00, 0x00 };

  /* Configure the SPI peripheral */
  if (ksz8851_interface_init(interface) == false)
  {
    return false;
  }

  // Init step1: read chip ID.
  /* Reset the Micrel in a proper state. */
  uint32_t TickResetMicrel = HAL_GetTick();
  do
  {
    /* Perform hardware reset */
    ksz8851_hard_reset(interface);

    ksz8851_soft_reset(interface, 0);

//    /* Perform Global Soft Reset (clear registers of PHY, MAC, QMU, DMA) */
//    ksz8851_reg_setbits(interface, REG_RESET_CTRL, GLOBAL_SOFTWARE_RESET);
//    HAL_Delay(1);
//    ksz8851_reg_clrbits(interface, REG_RESET_CTRL, GLOBAL_SOFTWARE_RESET);
//
//    /* Reset QMU Modules(flush out TXQ and RXQ) */
//    ksz8851_reg_setbits(interface, REG_RESET_CTRL, QMU_SOFTWARE_RESET);
//    HAL_Delay(1);
//    ksz8851_reg_clrbits(interface, REG_RESET_CTRL, QMU_SOFTWARE_RESET);

    // Read the device chip ID
    uint32_t TickReadID = HAL_GetTick();
    do
    {
      // Read the device chip ID, make sure it is correct ID 0x8872;
      // otherwise there are some errors on the host bus interface.
      // dev_id = 0x8872
      // Bit 15-8 = Chip family ID, 0x88
      // Bit 7-4 = Chip ID, 0x7 is assigned to KSZ8851SNL
      // Bit 3-1 = Revision ID, 0x2 >> 1 = 1
      // Bit 0 = Reserved
      dev_id = ksz8851_reg_read(interface, REG_CHIP_ID); /* CIDER */
      dmc_puts(TERMINAL_YELLOW);
      dmc_puts("Chip ID: ");
      dmc_puthex4cr(dev_id);
      dmc_puts(TERMINAL_DEFAULT);

//      if ((dev_id & 0xFFF0) != CHIP_ID_8851_16)
//      {
//        dmc_puts("Expected Device ID 0x");
//        dmc_puthex4(CHIP_ID_8851_16);
//        dmc_puts(", got 0x");
//        dmc_puthex4cr(dev_id);
//      }
//      else
//      {
//        dmc_puts("Device ID: 0x");
//        dmc_puthex4cr(dev_id);
//      }
      // Try for 10mS maximum
      if (HAL_GetTick() - TickReadID > 10)
      {
        break;
      }
    }
    while ((dev_id & 0xFFF0) != CHIP_ID_8851_16);

    // Try for 100mS maximum
    if (HAL_GetTick() - TickResetMicrel > 100)
    {
      dmc_puts(TERMINAL_LIGHT_RED);
      dmc_puts("FAILED: wrong Chip ID: ");
      dmc_puthex4cr(dev_id);
      dmc_puts(TERMINAL_DEFAULT);
//      return 1;
    }
  }
  while ((dev_id & 0xFFF0) != CHIP_ID_8851_16);
  dmc_puts(TERMINAL_LIGHT_GREEN);
  dmc_puts("Device ID: ");
  dmc_puthex4cr(dev_id);
  dmc_puts(TERMINAL_DEFAULT);

//    dmc_puts("MAC_ADDR: ");
//    dmc_puthex2(interface->MAC_address[0]);
//    dmc_puts(":");
//    dmc_puthex2(interface->MAC_address[1]);
//    dmc_puts(":");
//    dmc_puthex2(interface->MAC_address[2]);
//    dmc_puts(":");
//    dmc_puthex2(interface->MAC_address[3]);
//    dmc_puts(":");
//    dmc_puthex2(interface->MAC_address[4]);
//    dmc_puts(":");
//    dmc_puthex2cr(interface->MAC_address[5]);

  /* Init step2-4: Write QMU MAC address (low, middle then high). */
  // Write QMU MAC address (low)
  ksz8851_reg_write(interface, REG_MAC_ADDR_0, (interface->MAC_address[4] << 8) | interface->MAC_address[5]); /* MARL */
  // Write QMU MAC address (Medium)
  ksz8851_reg_write(interface, REG_MAC_ADDR_2, (interface->MAC_address[2] << 8) | interface->MAC_address[3]); /* MARM */
  // Write QMU MAC address (High)
  ksz8851_reg_write(interface, REG_MAC_ADDR_4, (interface->MAC_address[0] << 8) | interface->MAC_address[1]); /* MARH */

  // Just checking...
  uint16_t addr_0 = ksz8851_reg_read(interface, REG_MAC_ADDR_0);
  uint16_t addr_2 = ksz8851_reg_read(interface, REG_MAC_ADDR_2);
  uint16_t addr_4 = ksz8851_reg_read(interface, REG_MAC_ADDR_4);
  dmc_puts("MAC_ADDR: ");
  dmc_puthex2(addr_4 >> 8);
  dmc_puts(":");
  dmc_puthex2(addr_4 & 0xff);
  dmc_puts(":");
  dmc_puthex2(addr_2 >> 8);
  dmc_puts(":");
  dmc_puthex2(addr_2 & 0xff);
  dmc_puts(":");
  dmc_puthex2(addr_0 >> 8);
  dmc_puts(":");
  dmc_puthex2cr(addr_0 & 0xff);

  /* Init step5: Enable QMU Transmit Frame Data Pointer Auto Increment. */
  ksz8851_reg_write(interface, REG_TX_ADDR_PTR,  /* TXFDPR */
      ADDR_PTR_AUTO_INC);     /* Enable Frame data pointer increments automatically */
//    uint16_t TX_ADDR_PTR = ksz8851_reg_read(interface, REG_TX_ADDR_PTR);
//    dmc_puts("TX_ADDR_PTR: ");
//    dmc_puthex4cr(TX_ADDR_PTR);

  /* Enable QMU TxQ Auto-Enqueue frame */
//  ksz8851_reg_write(interface, REG_TXQ_CMD, TXQ_AUTO_ENQUEUE);

  /* Init step6: Configure QMU transmit control register. */
  /* Enable QMU Transmit:
   * Flow control,
   * Transmit padding,
   * Transmit CRC,
   * and IP/TCP/UDP/ICMP checksum generation.
   */
  // Packets shorter than 64 bytes are padded and the CRC is automatically generated
  ksz8851_reg_write(interface, REG_TX_CTRL,    /* TXCR */
      TX_CTRL_ICMP_CHECKSUM |   /* Enable ICMP frame checksum generation */
      TX_CTRL_UDP_CHECKSUM |    /* Enable UDP frame checksum generation */
      TX_CTRL_TCP_CHECKSUM |    /* Enable TCP frame checksum generation */
      TX_CTRL_IP_CHECKSUM |     /* Enable IP frame checksum generation */
      TX_CTRL_FLOW_ENABLE |     /* Enable transmit flow control */
      TX_CTRL_PAD_ENABLE |      /* Eanble adding a padding to a packet shorter than 64 bytes */
      TX_CTRL_CRC_ENABLE);      /* Enable adding a CRC to the end of transmit frame */

  /* Init step7: Enable QMU Receive Frame Data Pointer Auto Increment. */
  ksz8851_reg_write(interface, REG_RX_ADDR_PTR,  /* RXFDPR */
      ADDR_PTR_AUTO_INC);     /* Enable Frame data pointer increments automatically */

  /* Init step8: Configure QMU Receive Frame Threshold for one frame. */
  ksz8851_reg_write(interface, REG_RX_FRAME_CNT_THRES, 1); /* RXFCTFC */

  /* Init step9: Configure QMU receive control register1. */
  /* Enable QMU Receive:
   * Flow control,
   * Receive all broadcast frames,
   * receive unicast frames, and
   * IP/TCP/UDP checksum verification.
   */
  ksz8851_reg_write(interface, REG_RX_CTRL1,   /* RXCR1 */
      RX_CTRL_UDP_CHECKSUM |    /* Enable UDP frame checksum verification */
      RX_CTRL_TCP_CHECKSUM |    /* Enable TCP frame checksum verification */
      RX_CTRL_IP_CHECKSUM |   /* Enable IP frame checksum verification */
      RX_CTRL_MAC_FILTER |    /* Receive with address that pass MAC address filtering */
      RX_CTRL_FLOW_ENABLE |   /* Enable receive flow control */
      RX_CTRL_BROADCAST |     /* Receive all the broadcast frames */
      RX_CTRL_ALL_MULTICAST |   /* Receive all the multicast frames (including broadcast frames) */
      RX_CTRL_UNICAST |     /* Receive unicast frames that match the device MAC address */
      RX_CTRL_PROMISCUOUS);   /* Receive all incoming frames, regardless of frame's DA */

  /* Init step10: Configure QMU receive control register2. */
  /* Enable QMU Receive:
   * ICMP frame checksum verification,
   * UDP Lite frame checksum verification/generation,
   * IPv4/IPv6 UDP fragment frame pass
   * IPv4/IPv6 UDP UDP checksum field is zero pass,
   * single frame data burst if SPI master controller could read a single frame data in one SPI CSN activate,
   * and IPv6 UDP fragment frame pass.
   */
  ksz8851_reg_write(interface, REG_RX_CTRL2,     /* RXCR2 */
      RX_CTRL_IPV6_UDP_NOCHECKSUM | /* No checksum generation and verification if IPv6 UDP is fragment */
      RX_CTRL_UDP_LITE_CHECKSUM |   /* Enable UDP Lite frame checksum generation and verification */
      RX_CTRL_ICMP_CHECKSUM |     /* Enable ICMP frame checksum verification */
      RX_CTRL_BURST_LEN_FRAME);

  /* Init step11: Configure QMU receive queue: trigger INT and auto-dequeue frame. */
  /* Enable QMU Receive:
   * IP Header Two-Byte Offset,
   * Receive Frame Count Threshold,
   * and RXQ Auto-Dequeue frame.
   */
  ksz8851_reg_write(interface, REG_RXQ_CMD,      /* RXQCR */
      RXQ_EN_ON_FRAME_CNT_INT |   /* Enable RX interrupt by frame count threshold */
      RXQ_TWOBYTE_OFFSET |      /* Enable adding 2-bytes offset before IP frame header */
      RXQ_AUTO_DEQUEUE);        /* Enable Auto Dequeue RXQ Frame */

  /* Init step12: adjust SPI data output delay. */
  ksz8851_reg_write(interface, REG_BUS_CLOCK_CTRL, /* OBCR */
      BUS_CLOCK_166 |         /* 166 MHz on-chip bus clock (defaul is 125MHz) */
      BUS_CLOCK_DIVIDEDBY_1);     /* Bus clock devided by 1 */

  /* Init step13: Restart Port 1 auto-negotiation. */
  ksz8851_reg_setbits(interface, REG_PORT_CTRL,    /* P1CR */
      PORT_AUTO_NEG_RESTART);     /* Restart auto-negotiation */

  /* Init step13.1: Force link in half duplex if auto-negotiation failed. */
  if ((ksz8851_reg_read(interface, REG_PORT_CTRL) & PORT_AUTO_NEG_RESTART) != PORT_AUTO_NEG_RESTART) /* P1CR */
  {
    ksz8851_reg_clrbits(interface, REG_PORT_CTRL,  /* P1CR */
        PORT_FORCE_FULL_DUPLEX);  /* Force PHY in full duplex mode */
//    dmc_puts("Full Duplex\n");
  }

  /* Init step14: Clear interrupt status. */
  ksz8851_reg_write(interface, REG_INT_STATUS, 0xFFFF);  /* ISR */

  /* Init step14.1: Configure Low Watermark to 6KByte available buffer space out of 12KByte. */
  ksz8851_reg_write(interface, REG_RX_LOW_WATERMARK,   /* FCLWR */
      WATERMARK_6KB);           /* 6KByte Watermark */

  /* Init step14.2: Configure High Watermark to 4KByte available buffer space out of 12KByte. */
  ksz8851_reg_write(interface, REG_RX_HIGH_WATERMARK,  /* FCHWR */
      WATERMARK_4KB);           /* 4KByte Watermark */

  /* Init step15: Set interrupt mask. */
  /*
   * Enable Link Change\Transmit\Receive if your host processor can handle the interrupt,
   * otherwise do not need to do this step.
   */
  ksz8851_IntEnable(interface);

  /* Init step16: Enable QMU Transmit. */
  ksz8851_reg_setbits(interface, REG_TX_CTRL,  /* TXCR */
      TX_CTRL_ENABLE);      /* Enable tranmsit */

  /* Init step17: Enable QMU Receive. */
  ksz8851_reg_setbits(interface, REG_RX_CTRL1, /* RXCR1 */
      RX_CTRL_ENABLE);      /* Enable receive */


#if MULTI_FRAME
  /* Configure Receive Frame Threshold for 4 frames */
  ksz8851_reg_write(interface, REG_RX_FRAME_CNT_THRES, 4);

  /* Configure Receive Duration Threshold for 1 ms */
  ksz8851_reg_write(interface, REG_RX_TIME_THRES, 0x03e8);

  /* Enable
   * QMU Recieve IP Header Two-Byte Offset,
   * Receive Frame Count Threshold,
   * Receive Duration Timer Threshold, and
   * RXQ Auto-Dequeue frame.
   */
  ksz8851_reg_write(interface, REG_RXQ_CMD,
                RXQ_TWOBYTE_OFFSET |
                RXQ_TIME_INT | RXQ_FRAME_CNT_INT | RXQ_AUTO_DEQUEUE);
#endif

  return true;
}

/**
 * \brief Perform KSZ8851 hard reset
 * \param None
 * \return None
 */
void ksz8851_hard_reset(struct KSZ8851_INTERFACE *interface)
{
#if (KSZ8851_USE_HARD_RESET == 1)
  /* Perform hardware reset */
  if ((interface->rst_port) && (interface->rst_pin))
  {
    /* rst = low */
    HAL_GPIO_WritePin((GPIO_TypeDef*) interface->rst_port, interface->rst_pin, GPIO_PIN_RESET);
    HAL_Delay(1);
    /* rst = high */
    HAL_GPIO_WritePin((GPIO_TypeDef*) interface->rst_port, interface->rst_pin, GPIO_PIN_SET);
    HAL_Delay(1);
  }
#endif
}

/**
 * \brief Perform KSZ8851 soft reset
 * \param[in] Software reset QMU or GLOBAL
 * \return None
 */
void ksz8851_soft_reset(struct KSZ8851_INTERFACE *interface, uint8_t queue_only)
{
  if (queue_only)
  {
    /* Reset QMU Modules(flush out TXQ and RXQ) */
    ksz8851_reg_setbits(interface, REG_RESET_CTRL, QMU_SOFTWARE_RESET);
    HAL_Delay(1);
    ksz8851_reg_clrbits(interface, REG_RESET_CTRL, QMU_SOFTWARE_RESET);
  }
  else
  {
    /* Perform Global Soft Reset (clear registers of PHY, MAC, QMU, DMA) */
    ksz8851_reg_setbits(interface, REG_RESET_CTRL, GLOBAL_SOFTWARE_RESET);
    HAL_Delay(1);
    ksz8851_reg_clrbits(interface, REG_RESET_CTRL, GLOBAL_SOFTWARE_RESET);
  }
  HAL_Delay(1);
}

/**
 * \brief Read KSZ8851 register
 * \param[in] address Register address
 * \return Register value
 */
uint16_t ksz8851_reg_read(struct KSZ8851_INTERFACE *interface, uint16_t reg)
{
  uint8_t inbuf[4];
  uint8_t outbuf[4];
  uint16_t cmd = 0;
  uint16_t rddata = 0;

  /* Pull the CS pin low */
  HAL_GPIO_WritePin((GPIO_TypeDef*) interface->cs_port, interface->cs_pin, GPIO_PIN_RESET);

  /* Move register address to cmd bits 9-2 */
  cmd = (reg << 2) & REG_ADDR_MASK; // & 0x3F0

  /* Add byte enables to cmd */
  /* Last 2 bits still under "don't care bits" handled with byte enable. */
  /* Select byte enable for command. */
  if (reg & 2)
  {
    /* Odd word address writes bytes 2 and 3 */
    cmd |= (0xc << 10);
  }
  else
  {
    /* Even word address write bytes 0 and 1 */
    cmd |= (0x3 << 10);
  }

  /* Add opcode to cmd */
  /* Add command read code. */
  cmd |= CMD_READ;
  outbuf[0] = cmd >> 8;
  outbuf[1] = cmd & 0xff;
  outbuf[2] = 0xff;
  outbuf[3] = 0xff;

  /* Perform blocking SPI transfer. Discard function returned value! TODO: handle it? */
  (void) HAL_SPI_TransmitReceive(interface->hspi, (uint8_t*) outbuf, (uint8_t *) inbuf, 4, 5000);

  /* Terminate the operation by raising the CS pin */
  HAL_GPIO_WritePin((GPIO_TypeDef*) interface->cs_port, interface->cs_pin, GPIO_PIN_SET);

  rddata = (inbuf[3] << 8) | inbuf[2];
  return rddata;
}

/**
 * \brief Write KSZ8851 register
 * \param[in] address Register address
 * \param[in] data Register value
 */
void ksz8851_reg_write(struct KSZ8851_INTERFACE *interface, uint16_t reg, uint16_t wrdata)
{
  uint8_t outbuf[4];
  uint16_t cmd = 0;

  /* Pull the CS pin low */
  HAL_GPIO_WritePin((GPIO_TypeDef*) interface->cs_port, interface->cs_pin, GPIO_PIN_RESET);

  /* Move register address to cmd bits 9-2 */
  cmd = (reg << 2) & REG_ADDR_MASK;
  // 0x90 << 2 = 0x240 & 0x3F0 = 0x240


  /* Add byte enables to cmd */
  /* Last 2 bits still under "don't care bits" handled with byte enable. */
  /* Select byte enable for command. */
  if (reg & 2)
  {
    /* Odd word address writes bytes 2 and 3 */
    // 0x3000 0011 0000 0000 0000
    cmd |= (0xc << 10);
  }
  else
  {
    /* Even word address write bytes 0 and 1 */
    // 0x0C00 0000 1100 0000 0000
    cmd |= (0x3 << 10);
  }

  /* Add opcode to cmd */
  /* Add command write code. */
  cmd |= CMD_WRITE;
  outbuf[0] = cmd >> 8;
  outbuf[1] = cmd & 0xff;
  outbuf[2] = wrdata & 0xff;
  outbuf[3] = wrdata >> 8;

  /* Perform blocking SPI transfer. Discard function returned value! TODO: handle it? */
  (void) HAL_SPI_Transmit(interface->hspi, (uint8_t*) outbuf, 4, 2000);

  /* Terminate the operation by raising the CS pin */
  HAL_GPIO_WritePin((GPIO_TypeDef*) interface->cs_port, interface->cs_pin, GPIO_PIN_SET);
}

/**
 * \brief Read register content, set bitmask and write back to register.
 * \param reg the register address to modify.
 * \param bits_to_set bitmask to apply.
 */
void ksz8851_reg_setbits(struct KSZ8851_INTERFACE *interface, uint16_t reg, uint16_t bits_to_set)
{
  uint16_t temp;

  // In order to set a bit in a register, such as during initialization,
  // read the register first, modify the target bit only and write it back.
  temp = ksz8851_reg_read(interface, reg);
  temp |= bits_to_set;
  ksz8851_reg_write(interface, reg, temp);
}

/**
 * \brief Read register content, clear bitmask and write back to register.
 * \param reg the register address to modify.
 * \param bits_to_set bitmask to apply.
 */
void ksz8851_reg_clrbits(struct KSZ8851_INTERFACE *interface, uint16_t reg, uint16_t bits_to_clr)
{
  uint16_t temp;

  // In order to set a bit in a register, such as during initialization,
  // read the register first, modify the target bit only and write it back.
  temp = ksz8851_reg_read(interface, reg);
  temp &= ~bits_to_clr;
  ksz8851_reg_write(interface, reg, temp);
}

void ksz8851_IntEnable(struct KSZ8851_INTERFACE *interface)
{
  /* Enable interrupts */
  ksz8851_reg_write(interface, REG_INT_ENABLE,     /* IER */
      KSZ8851SNL_INT_LINK_CHANGE |
      KSZ8851SNL_INT_RX_DONE |    /** Enable transmit done interrupt */
      KSZ8851SNL_INT_RX_STOPPED |   /** Enable receive process stopped interrupt */
      KSZ8851SNL_INT_TX_STOPPED |   /** Enable transmit process stopped interrupt */
      KSZ8851SNL_INT_LINK_CHANGE |  /** Enable link change interrupt */
      KSZ8851SNL_INT_RX_SPI_ERROR); /** Enable receive SPI bus error interrupt */
}

void ksz8851_IntDisable(struct KSZ8851_INTERFACE *interface)
{
  ksz8851_reg_write(interface, REG_INT_ENABLE, INT_NO_INT);
}

void ksz8851_IntClearAll(struct KSZ8851_INTERFACE *interface)
{
  ksz8851_reg_write(interface, REG_INT_STATUS, 0xFFFF);  /* ISR */
}

void ksz8851_IntClearFlags(struct KSZ8851_INTERFACE *interface, uint16_t flags)
{
  ksz8851_reg_write(interface, REG_INT_STATUS, flags);
}

void ksz8851_set_pm(struct KSZ8851_INTERFACE *interface, uint8_t mode)
{
  uint8_t pmecr;

  pmecr = ksz8851_reg_read(interface, REG_POWER_CNTL);
  pmecr &= ~POWER_STATE_MASK;
  pmecr |= mode;
  ksz8851_reg_write(interface, REG_POWER_CNTL, pmecr);
}

// https://github.com/fabiobaltieri/avr-nrf/blob/master/firmware/ksz8851snl.c
void ksz8851_send_packet(struct KSZ8851_INTERFACE *interface, uint8_t *buf, uint16_t length)
{
  ksz8851_reg_write(interface, REG_RXQ_CMD, interface->rxqcr | RXQ_START);
  ksz8851_fifo_write(interface, buf, length);
  ksz8851_reg_write(interface, REG_RXQ_CMD, interface->rxqcr);
  ksz8851_reg_write(interface, REG_TXQ_CMD, RXQ_CMD_FREE_PACKET);
//  led_net_tx();
}

uint16_t ksz8851_read_packet(struct KSZ8851_INTERFACE *interface, uint8_t *buf, uint16_t limit)
{
  uint16_t rxlen;
  uint16_t rxfctr;
  uint16_t ret = 0;
  uint8_t i;

//  interface->rxqcr = 0;  // Jack: Additional bits in REG_RXQ_CMD


  // Bit 15-8
  // RXFC RX Frame Count
  // To indicate the total received frames in RXQ frame buffer when receive
  // interrupt (bit13=1 in ISR) occurred and write “1” to clear this bit 13 in
  // ISR. The host CPU can start to read the updated receive frame header
  // information in RXFHSR/RXFHBCR registers after read this RX frame
  // count register.
  rxfctr = ksz8851_reg_read(interface, KS_RXFCTR);
  rxfctr = rxfctr >> 8;
  dmc_puts("rxfctr: ");
  dmc_putintcr(rxfctr);

  for (i = 0; i < rxfctr; i++)
  {
    // Bits 11-0 (0000 1111 1111 1111)
    // RXBC Receive Byte Count
    // This field indicates the present received frame byte size.
    rxlen = ksz8851_reg_read(interface, REG_RX_FHR_BYTE_CNT) & 0x0fff; // 12 bits
//    dmc_puts("rxlen: ");
//    dmc_putintcr(rxlen);

    // RXFPAI RX Frame Pointer Auto Increment
    // When this bit is set, the RXQ Address register increments automatically
    // on accesses to the data register. The increment is by one for every byte
    // access, by two for every word access, and by four for every double word
    // access.
    // When this bit is reset, the RX frame data pointer is manually controlled
    // by user to access the RX frame location.
    // Bit 10-0, RXFP RX Frame Pointer
    // RX Frame data pointer index to the Data register for access.
    // This pointer value must reset to 0x000 before each DMA operation from
    // the host CPU to read RXQ frame buffer.
    ksz8851_reg_write(interface, REG_RX_ADDR_PTR, ADDR_PTR_AUTO_INC | 0x0000);
    // RXQ COMMAND REGISTER
    // RXQ_START
    // SDA Start DMA Access
    // When this bit is written as 1, the KSZ8851SNL allows a DMA operation
    // from the host CPU to access either read RXQ frame buffer or write TXQ
    // frame buffer with SPI command operation for RXQ/TXQ FIFO read/
    // write. All registers access are disabled except this register during this
    // DMA operation.
    // This bit must be set to 0 when DMA operation is finished in order to
    // access the rest of registers.
    // ADRFE Auto-Dequeue RXQ Frame Enable
    // When this bit is written as 1, the KSZ8851SNL will automatically enable
    // RXQ frame buffer dequeue. The read pointer in RXQ frame buffer will be
    // automatically adjusted to next received frame location after current
    // frame is completely read by the host.
    interface->rxqcr = ksz8851_reg_read(interface, REG_RXQ_CMD);
    ksz8851_reg_write(interface, REG_RXQ_CMD, interface->rxqcr | RXQ_START | RXQ_AUTO_DEQUEUE);

    if (rxlen > 4)
    {
      if (i == 0 && rxlen <= limit)
      {
        rxlen -= rxlen % 4;
        ksz8851_fifo_read(interface, buf, rxlen);
        ret = rxlen;
      }
      else
      {
        rxlen -= rxlen % 4;
        ksz8851_fifo_read(interface, NULL, rxlen);
        ret = 0;
      }
    }

    ksz8851_reg_write(interface, REG_RXQ_CMD, interface->rxqcr);
  }

//  led_net_rx();

  return ret;
}

uint8_t ksz8851_has_data(struct KSZ8851_INTERFACE *interface)
{
  uint16_t isr = ksz8851_reg_read(interface, REG_INT_STATUS);

  ksz8851_reg_write(interface, REG_INT_STATUS, isr);
  if (isr & IRQ_RXI)
  {
    return 1;
  }
  return 0;
}

/* ------------------- Exported functions -------------------- */
uint8_t ksz8851_get_spi_state(struct KSZ8851_INTERFACE *interface)
{
  HAL_SPI_StateTypeDef spiState;
  uint8_t spiIntCode;

  spiState = HAL_SPI_GetState(interface->hspi);

  switch (spiState)
  {
  case HAL_SPI_STATE_READY:
    spiIntCode = INT_SPI_STATE_READY;
    break;
  case HAL_SPI_STATE_BUSY:
  case HAL_SPI_STATE_BUSY_TX:
  case HAL_SPI_STATE_BUSY_RX:
  case HAL_SPI_STATE_BUSY_TX_RX:
    spiIntCode = INT_SPI_STATE_BUSY;
    break;
  case HAL_SPI_STATE_ERROR:
  default:
    //TODO: get and understand the error code
    //error_code = HAL_SPI_GetError(hspi);
    spiIntCode = INT_SPI_STATE_ERROR;
  }

  return spiIntCode;
}

uint16_t ksz8851_fifo_read(struct KSZ8851_INTERFACE *interface, uint8_t *buf, uint16_t len)
{
  uint8_t tmp_buf[9];
  uint8_t pad_bytes;

  // TODO: check len value
  if (len == 0)
  {
    return;
  }

  /* Pull the CS pin low */
  HAL_GPIO_WritePin((GPIO_TypeDef*) interface->cs_port, interface->cs_pin, GPIO_PIN_RESET);

  /* Calculate number of dummy pad bytes to read a 32-bits aligned buffer */
  pad_bytes = ((len & 0x03) != 0) ? (4 - (len & 0x03)) : 0;

  // The first byte: Command phase
  tmp_buf[0] = FIFO_READ;
  // The first 4 bytes are dummy data and must be discarded
  // The first 4 bytes are the RX packet header, ignore

  /* Perform blocking SPI transfer. */
  (void) HAL_SPI_TransmitReceive(interface->hspi, (uint8_t*) tmp_buf, (uint8_t*) tmp_buf, 11, 2000);

//  for (uint8_t i = 0; i < 11; i++)
//  {
//    dmc_puthex2(tmp_buf[i]);
//    dmc_putc(' ');
//  }
//  dmc_putcr();

 // (void) HAL_SPI_Receive(interface->hspi, (uint8_t*) tmp_buf, 2, 2000);

  /* Maintain alignment to 4-byte boundaries */
  /* update length to a 32-bits aligned value */
  len += pad_bytes;

  //Data phase
  /* ATTENTION: pad bytes are the bytes beyond buffer length (can be any rubbish value) */
#if (KSZ8851_USE_TX_DMA == 0)
  /* Perform non-blocking DMA SPI transfer. Discard function returned value! TODO: handle it? */
  (void) HAL_SPI_Receive(interface->hspi, (uint8_t*) buf, len, 2000);
#else
  // https://community.st.com/s/article/FAQ-DMA-is-not-working-on-STM32H7-devices
  clr_dma_rx_ended(interface);
  /* Invalidate D-cache before reception */
  /* Make sure the address is 32-byte aligned and add 32-bytes to length, in case it overlaps cacheline */
//  SCB_InvalidateDCache_by_Addr((uint32_t*)(((uint32_t)rx_buffer) & ~(uint32_t)0x1F), RX_LENGTH+32);
  SCB_InvalidateDCache_by_Addr((uint32_t*)(((uint32_t)buf) & ~(uint32_t)0x1F), len+32);

  __DSB();
  /* Perform non-blocking DMA SPI transfer. Discard function returned value! TODO: handle it? */
  (void) HAL_SPI_Receive_DMA(interface->hspi, (uint8_t*) buf, len);
  /* an interrupt will occur */
  dmc_puts("Done\n");
  wait_dma_rx_ended(interface);
  dmc_puts("Done\n");
#endif

  // Calculate bytes received from packets
  if ((buf[ETH_HDR_TYPE] == 0x08) && (buf[ETH_HDR_TYPE + 1] == 0x00)) // TCP/UDP
  {
    if (buf[IP_HDR_OFFSET + IP_HDR_PROTOCOL] == 0x11) // UDP
    {
    }
//    for (uint8_t i = 0; i < len; i++)
//    {
//      dmc_puthex2(buf[i]);
//      dmc_putc(' ');
//    }
//    dmc_putcr();
////    len = ((buf[IP_HDR_OFFSET + IP_HDR_TOTAL_LENGTH] << 8) || buf[IP_HDR_OFFSET + IP_HDR_TOTAL_LENGTH + 1]) + ETH_HDR_SIZE;
//    dmc_puthex2(buf[IP_HDR_OFFSET + IP_HDR_TOTAL_LENGTH]);
//    dmc_puthex2(buf[IP_HDR_OFFSET + IP_HDR_TOTAL_LENGTH + 1]);
//    dmc_putcr();
//    dmc_putint(len);
//    dmc_putcr();
  }
  if ((buf[ETH_HDR_TYPE] == 0x08) && (buf[ETH_HDR_TYPE] == 0x06)) // ARP
  {
    len = 60;
  }

  /* Terminate the operation by raising the CS pin */
  HAL_GPIO_WritePin((GPIO_TypeDef*) interface->cs_port, interface->cs_pin, GPIO_PIN_SET);

  return len;
}

void ksz8851_fifo_write(struct KSZ8851_INTERFACE *interface, uint8_t *buf, uint16_t len)
{
  uint8_t outbuf[5];
  uint8_t pad_bytes;
  static uint8_t frameID = 0;

  /* Pull the CS pin low */
  HAL_GPIO_WritePin((GPIO_TypeDef*) interface->cs_port, interface->cs_pin, GPIO_PIN_RESET);

  // TODO: check len value
  if (len == 0)
  {
    return;
  }

  /* length is 11 bits long */
  len &= 0x07FF;

  /* Calculate number of dummy pad bytes to send a 32-bits aligned buffer */
  pad_bytes = ((len & 0x03) != 0) ? (4 - (len & 0x03)) : 0;

  // Command phase
  /* Prepare control word and byte count. */
  outbuf[0] = FIFO_WRITE;
  outbuf[1] = (frameID++ & FRAME_ID_MASK);
  outbuf[2] = 0;
  /* Write txPacketLength to the "byte count" of the frame header. */
  outbuf[3] = len & LSB_MASK;
  outbuf[4] = len >> MSB_POS;

  /* Perform blocking SPI transfer. */
  (void) HAL_SPI_Transmit(interface->hspi, (uint8_t*) outbuf, 5, 2000);

  /* update length to a 32-bits aligned value */
  len += pad_bytes;

  //Data phase
  /* ATTENTION: pad bytes are the bytes beyond buffer length (can be any rubbish value) */
#if (KSZ8851_USE_TX_DMA == 0)
  /* Perform blocking DMA SPI transfer. Discard function returned value! TODO: handle it? */
  (void) HAL_SPI_TransmitReceive(interface->hspi, (uint8_t*) buf, (uint8_t*) buf, len, 2000);
#else
  // https://community.st.com/s/article/FAQ-DMA-is-not-working-on-STM32H7-devices
  clr_dma_tx_ended(interface);
  /* Clean D-cache */
  /* Make sure the address is 32-byte aligned and add 32-bytes to length, in case it overlaps cacheline */
//  SCB_CleanDCache_by_Addr((uint32_t*)(((uint32_t)tx_buffer) & ~(uint32_t)0x1F), TX_LENGTH+32);
  SCB_CleanDCache_by_Addr((uint32_t*)(((uint32_t)buf) & ~(uint32_t)0x1F), len+32);

  __DSB();
  /* Perform non-blocking DMA SPI transfer. Discard function returned value! TODO: handle it? */
  (void) HAL_SPI_TransmitReceive_DMA(interface->hspi, (uint8_t*) buf, (uint8_t*) buf, len);
  /* an interrupt will occur */
  dmc_puts("Done\n");
  wait_dma_tx_ended(interface);
  dmc_puts("Done\n");
#endif

  /* Terminate the operation by raising the CS pin */
  HAL_GPIO_WritePin((GPIO_TypeDef*) interface->cs_port, interface->cs_pin, GPIO_PIN_SET);
}

uint32_t ksz8851_MIBCountersRead(struct KSZ8851_INTERFACE *interface, uint16_t offset)
{
  uint16_t data;
  uint32_t counter;

  data = MIB_MASK | offset;
  ksz8851_reg_write(interface, REG_IND_IACR, data);
  counter = ksz8851_reg_read(interface, REG_IND_DATA_LOW);
  counter |= ksz8851_reg_read(interface, REG_IND_DATA_HIGH) << 16;

  return counter;
}

/***************************************************************************//**
 * @brief Dumps the Management Information Base Counters
 * @note  Support method used for debugging.
 *
 * @param param
 *     the string representing the moment of the register dump
 *
 *****************************************************************************/
void KSZ8851SNL_ReadMIBCounters(struct KSZ8851_INTERFACE *interface, char* param)
{
//  EFM_ASSERT(param != NULL);
//  uint16_t data;
  uint32_t counter;
  dmc_puts("Dumping MIB Counters values @");
  dmc_puts(param);
  dmc_puts("\n");
  int i;
  for (i = 0; i < 0x20; i++)
  {
    counter = ksz8851_MIBCountersRead(interface, i);
    dmc_puts("MIB_REG[");
    dmc_puthex2(i);
    dmc_puts("] contains ");
    dmc_puthex4(counter);
    dmc_puts("\n");
  }
}

void ksz8851_AllRegistersDump(struct KSZ8851_INTERFACE *interface)
{
  dmc_puts("###################### ALL REGISTER DUMP ########################\n");
  int i;
  for (i = 0x00; i < 0xFF; i += 0x02)
  {
    if ((i % 8 == 0) && (i > 0))
    {
      dmc_puts("\n");
    }
    dmc_puts("REG[0x");
    dmc_puthex2(i);
    dmc_puts("]=0x");
    dmc_puthex4cr(ksz8851_reg_read(interface, i));
  }
  dmc_puts("\n");
  dmc_puts("#################################################################\n");
}

void ksz8851_RegistersDump(struct KSZ8851_INTERFACE *interface)
{
  dmc_puts("##################### SPECIAL REGISTER DUMP ######################\n");
//  printf("MARL  [0x%02X]=0x%04X\n", REG_MAC_ADDR_0, ksz8851_reg_read(interface, REG_MAC_ADDR_0));
  dmc_puts("MARL  [0x");
  dmc_puthex2(REG_MAC_ADDR_0);
  dmc_puts("]=0x");
  dmc_puthex4cr(ksz8851_reg_read(interface, REG_MAC_ADDR_0));
//  printf("MARM  [0x%02X]=0x%04X\n", REG_MAC_ADDR_2, ksz8851_reg_read(interface, REG_MAC_ADDR_2));
  dmc_puts("MARM  [0x");
  dmc_puthex2(REG_MAC_ADDR_2);
  dmc_puts("]=0x");
  dmc_puthex4cr(ksz8851_reg_read(interface, REG_MAC_ADDR_2));
//  printf("MARH  [0x%02X]=0x%04X\n", REG_MAC_ADDR_4, ksz8851_reg_read(interface, REG_MAC_ADDR_4));
  dmc_puts("MARH  [0x");
  dmc_puthex2(REG_MAC_ADDR_4);
  dmc_puts("]=0x");
  dmc_puthex4cr(ksz8851_reg_read(interface, REG_MAC_ADDR_4));
//  printf("OBCR  [0x%02X]=0x%04X\n", REG_BUS_CLOCK_CTRL, ksz8851_reg_read(interface, REG_BUS_CLOCK_CTRL));
  dmc_puts("OBCR  [0x");
  dmc_puthex2(REG_BUS_CLOCK_CTRL);
  dmc_puts("]=0x");
  dmc_puthex4cr(ksz8851_reg_read(interface, REG_BUS_CLOCK_CTRL));
//  printf("GRR   [0x%02X]=0x%04X\n", REG_RESET_CTRL, ksz8851_reg_read(interface, REG_RESET_CTRL));
  dmc_puts("GRR   [0x");
  dmc_puthex2(REG_RESET_CTRL);
  dmc_puts("]=0x");
  dmc_puthex4cr(ksz8851_reg_read(interface, REG_RESET_CTRL));
//  printf("TXCR  [0x%02X]=0x%04X\n", REG_TX_CTRL, ksz8851_reg_read(interface, REG_TX_CTRL));
  dmc_puts("TXCR  [0x");
  dmc_puthex2(REG_TX_CTRL);
  dmc_puts("]=0x");
  dmc_puthex4cr(ksz8851_reg_read(interface, REG_TX_CTRL));
//  printf("RXCR1 [0x%02X]=0x%04X\n", REG_RX_CTRL1, ksz8851_reg_read(interface, REG_RX_CTRL1));
  dmc_puts("RXCR1 [0x");
  dmc_puthex2(REG_RX_CTRL1);
  dmc_puts("]=0x");
  dmc_puthex4cr(ksz8851_reg_read(interface, REG_RX_CTRL1));
//  printf("RXCR2 [0x%02X]=0x%04X\n", REG_RX_CTRL2, ksz8851_reg_read(interface, REG_RX_CTRL2));
  dmc_puts("RXCR2 [0x");
  dmc_puthex2(REG_RX_CTRL2);
  dmc_puts("]=0x");
  dmc_puthex4cr(ksz8851_reg_read(interface, REG_RX_CTRL2));
//  printf("TXMIR [0x%02X]=0x%04X\n", REG_TX_MEM_INFO, ksz8851_reg_read(interface, REG_TX_MEM_INFO));
  dmc_puts("TXMIR [0x");
  dmc_puthex2(REG_TX_MEM_INFO);
  dmc_puts("]=0x");
  dmc_puthex4cr(ksz8851_reg_read(interface, REG_TX_MEM_INFO));
#if (READ_UNSAFE_REGISTERS)
//  printf("RXFHSR[0x%02X]=0x%04X\n", REG_RX_FHR_STATUS, ksz8851_reg_read(interface, REG_RX_FHR_STATUS));
  dmc_puts("RXFHSR[0x");
  dmc_puthex2(REG_RX_FHR_STATUS);
  dmc_puts("]=0x");
  dmc_puthex4cr(ksz8851_reg_read(interface, REG_RX_FHR_STATUS));
#endif
//  printf("TXQCR [0x%02X]=0x%04X\n", REG_TXQ_CMD, ksz8851_reg_read(interface, REG_TXQ_CMD));
  dmc_puts("TXQCR [0x");
  dmc_puthex2(REG_TXQ_CMD);
  dmc_puts("]=0x");
  dmc_puthex4cr(ksz8851_reg_read(interface, REG_TXQ_CMD));
//  printf("RXQCR [0x%02X]=0x%04X\n", REG_RXQ_CMD, ksz8851_reg_read(interface, REG_RXQ_CMD));
  dmc_puts("RXQCR [0x");
  dmc_puthex2(REG_RXQ_CMD);
  dmc_puts("]=0x");
  dmc_puthex4cr(ksz8851_reg_read(interface, REG_RXQ_CMD));
//  printf("TXFDPR[0x%02X]=0x%04X\n", REG_TX_ADDR_PTR, ksz8851_reg_read(interface, REG_TX_ADDR_PTR));
  dmc_puts("TXFDPR[0x");
  dmc_puthex2(REG_TX_ADDR_PTR);
  dmc_puts("]=0x");
  dmc_puthex4cr(ksz8851_reg_read(interface, REG_TX_ADDR_PTR));
//  printf("RXFDPR[0x%02X]=0x%04X\n", REG_RX_ADDR_PTR, ksz8851_reg_read(interface, REG_RX_ADDR_PTR));
  dmc_puts("RXFDPR[0x");
  dmc_puthex2(REG_RX_ADDR_PTR);
  dmc_puts("]=0x");
  dmc_puthex4cr(ksz8851_reg_read(interface, REG_RX_ADDR_PTR));
//  printf("IER   [0x%02X]=0x%04X\n", REG_INT_ENABLE, ksz8851_reg_read(interface, REG_INT_ENABLE));
  dmc_puts("IER   [0x");
  dmc_puthex2(REG_INT_ENABLE);
  dmc_puts("]=0x");
  dmc_puthex4cr(ksz8851_reg_read(interface, REG_INT_ENABLE));
//  printf("ISR   [0x%02X]=0x%04X\n", REG_INT_STATUS, ksz8851_reg_read(interface, REG_INT_STATUS));
  dmc_puts("ISR   [0x");
  dmc_puthex2(REG_INT_STATUS);
  dmc_puts("]=0x");
  dmc_puthex4cr(ksz8851_reg_read(interface, REG_INT_STATUS));
//  printf("RXFCTR[0x%02X]=0x%04X\n", KS_RXFCTR, ksz8851_reg_read(interface, KS_RXFCTR));
  dmc_puts("RXFCTR[0x");
  dmc_puthex2(KS_RXFCTR);
  dmc_puts("]=0x");
  dmc_puthex4cr(ksz8851_reg_read(interface, KS_RXFCTR));
#if (READ_UNSAFE_REGISTERS)
//  printf("TXNTFSR[0x%02X]=0x%04X\n", TXNTFSR, ksz8851_reg_read(interface, TXNTFSR));
  dmc_puts("TXNTFSR[0x");
  dmc_puthex2(REG_TX_MEM_INFO);
  dmc_puts("]=0x");
  dmc_puthex4cr(ksz8851_reg_read(interface, REG_TX_MEM_INFO));
#endif
  printf("CIDER [0x%02X]=0x%04X\n", REG_CHIP_ID, ksz8851_reg_read(interface, REG_CHIP_ID));
  dmc_puts("CIDER [0x");
  dmc_puthex2(REG_CHIP_ID);
  dmc_puts("]=0x");
  dmc_puthex4cr(ksz8851_reg_read(interface, REG_CHIP_ID));
//  printf("PHYRR [0x%02X]=0x%04X\n", REG_PHY_RESET, ksz8851_reg_read(interface, REG_PHY_RESET));
  dmc_puts("PHYRR [0x");
  dmc_puthex2(REG_PHY_RESET);
  dmc_puts("]=0x");
  dmc_puthex4cr(ksz8851_reg_read(interface, REG_PHY_RESET));
//  printf("P1MBCR[0x%02X]=0x%04X\n", REG_PHY_CNTL, ksz8851_reg_read(interface, REG_PHY_CNTL));
  dmc_puts("P1MBCR[0x");
  dmc_puthex2(REG_PHY_CNTL);
  dmc_puts("]=0x");
  dmc_puthex4cr(ksz8851_reg_read(interface, REG_PHY_CNTL));
//  printf("P1CR  [0x%02X]=0x%04X\n", REG_PORT_CTRL, ksz8851_reg_read(interface, REG_PORT_CTRL));
  dmc_puts("P1CR  [0x");
  dmc_puthex2(REG_PORT_CTRL);
  dmc_puts("]=0x");
  dmc_puthex4cr(ksz8851_reg_read(interface, REG_PORT_CTRL));
  dmc_puts("#################################################################\n");
}

uint16_t ksz8851_IntGet(struct KSZ8851_INTERFACE *interface)
{
  return ksz8851_reg_read(interface, REG_INT_STATUS);
}

void ksz8851_PMECRStatusClear(struct KSZ8851_INTERFACE *interface, uint16_t flags)
{
  uint16_t status = ksz8851_reg_read(interface, REG_POWER_CNTL) | flags;
  ksz8851_reg_write(interface, REG_POWER_CNTL, status);
}

uint16_t ksz8851_RXQCRGet(struct KSZ8851_INTERFACE *interface)
{
  return ksz8851_reg_read(interface, REG_RXQ_CMD);
}

uint16_t ksz8851_FrameCounterSet(struct KSZ8851_INTERFACE *interface)
{
  /* Read the frame count and threshold register */
  uint16_t rxftr = ksz8851_reg_read(interface, KS_RXFCTR);
  /* Extract the actual number of frames from RX_FRAME_THRES_REG */
  interface->rxFrameCount = rxftr >> MSB_POS;
  return rxftr;
}

void ksz8851_TxQueueReset(struct KSZ8851_INTERFACE *interface)
{
  uint16_t data;

  data = ksz8851_reg_read(interface, REG_TX_CTRL);
  /* Disable TX */
  data &= ~( ( uint16_t ) (TX_FLOW_CTRL_ENABLE | TX_FLOW_CTRL_FLUSH_QUEUE) );
  ksz8851_reg_write(interface, REG_TX_CTRL, data);
  HAL_Delay(2);
  /* Flush */
  data |= ( uint16_t ) TX_FLOW_CTRL_FLUSH_QUEUE;
  ksz8851_reg_write(interface, REG_TX_CTRL, data);
  HAL_Delay(1);
  /* normal op - no flush */
  data &= ~( uint16_t ) TX_FLOW_CTRL_FLUSH_QUEUE;
  ksz8851_reg_write(interface, REG_TX_CTRL, data);
  HAL_Delay(1);
  /* Enable TX */
  data |= ( uint16_t ) TX_FLOW_CTRL_ENABLE;
  ksz8851_reg_write(interface, REG_TX_CTRL, data);
  HAL_Delay(1);
}

void ksz8851_RxQueueReset(struct KSZ8851_INTERFACE *interface)
{
  uint16_t data;

  data = ksz8851_reg_read(interface, REG_RX_CTRL1);
  /* Disable RX */
  data &= ~( ( uint16_t ) (RX_FLOW_CTRL_RX_ENABLE | RX_FLOW_CTRL_FLUSH_QUEUE) );
  ksz8851_reg_write(interface, REG_RX_CTRL1, data);
  HAL_Delay(2);
  /* Flush */
  ksz8851_reg_write(interface, REG_RX_CTRL1, data);
  /* Clear flush */
  data &= ~( uint16_t ) RX_FLOW_CTRL_FLUSH_QUEUE;
  ksz8851_reg_write(interface, REG_RX_CTRL1, data);
  HAL_Delay(1);
  /* Write default config with enable set */
  data |= ( uint16_t ) RX_FLOW_CTRL_RX_ENABLE;
  ksz8851_reg_write(interface, REG_RX_CTRL1, data);
  HAL_Delay(1);
}

uint32_t ksz8851snl_reset_rx(struct KSZ8851_INTERFACE *interface)
{
  uint16_t usValue;

  usValue = ksz8851_reg_read(interface, REG_RX_CTRL1);
  usValue &= ~( ( uint16_t ) (RX_CTRL_ENABLE | RX_CTRL_FLUSH_QUEUE) );
  ksz8851_reg_write(interface, REG_RX_CTRL1, usValue );
//  HAL_Delay(2);
  ksz8851_reg_write(interface, REG_RX_CTRL1, usValue | RX_CTRL_FLUSH_QUEUE );
//  HAL_Delay(1);
  ksz8851_reg_write(interface, REG_RX_CTRL1, usValue );
//  HAL_Delay(1);
  ksz8851_reg_write(interface, REG_RX_CTRL1, usValue | RX_CTRL_ENABLE );
//  HAL_Delay(1);

  return ( uint32_t )usValue;
}

uint32_t ksz8851snl_reset_tx(struct KSZ8851_INTERFACE *interface)
{
  uint16_t usValue;

  usValue = ksz8851_reg_read(interface,  REG_TX_CTRL );
  usValue &= ~( ( uint16_t ) (TX_CTRL_ENABLE | TX_CTRL_FLUSH_QUEUE) );
  ksz8851_reg_write(interface, REG_TX_CTRL, usValue );
//  HAL_Delay(2);
  ksz8851_reg_write(interface, REG_TX_CTRL, usValue | TX_CTRL_FLUSH_QUEUE );
//  HAL_Delay(1);
  ksz8851_reg_write(interface, REG_TX_CTRL, usValue );
//  HAL_Delay(1);
  ksz8851_reg_write(interface, REG_TX_CTRL, usValue | TX_CTRL_ENABLE );
//  HAL_Delay(1);

  return ( uint32_t )usValue;
}

void ksz8851_Enable(struct KSZ8851_INTERFACE *interface)
{
  uint16_t data;

  ksz8851_reg_write(interface, REG_INT_ENABLE, KSZ8851SNL_INT_ENABLE_MASK);

  /* Enable QMU Transmit */
  data = ksz8851_reg_read(interface, REG_TX_CTRL);
  data |= TX_FLOW_CTRL_ENABLE;
  ksz8851_reg_write(interface, REG_TX_CTRL, data);

  /* Enable QMU Receive */
  data = ksz8851_reg_read(interface, REG_RX_CTRL1);
  data |= RX_FLOW_CTRL_RX_ENABLE;
  ksz8851_reg_write(interface, REG_RX_CTRL1, data);
}

bool ksz8851_TransmitBegin(struct KSZ8851_INTERFACE *interface, uint16_t length)
{
  uint16_t txmir;
  uint16_t data, reqSize;
  uint8_t outbuf[4];

  /* Wait for previous frame to finish before setting up a new one */
  while (ksz8851_reg_read(interface, REG_TXQ_CMD) & TXQ_ENQUEUE)
  {
    ;
  }

  /* Make sure there is room for
   *
   * 4 bytes Control Word
   * 4 bytes Byte Count
   * n bytes Packet
   * 4 bytes CRC
   */
  reqSize = length + 12;
  txmir = ksz8851_reg_read(interface, REG_TX_MEM_INFO) & TX_MEM_AVAIL_MASK;
//  LWIP_DEBUGF(NETIF_DEBUG, ("KSZ8851SNL_LongTransmitInit: txmir =%hu  reqSize = %hu \n", txmir, reqSize));

  if (txmir < reqSize)
  {
    /* TXQ is out of memory */
//    LWIP_DEBUGF(NETIF_DEBUG | LWIP_DBG_LEVEL_WARNING,
//        ("Not enough TXQ Memory, available=%u required=%u\n", txmir, reqSize));
    return false;
  }

//  LWIP_DEBUGF(NETIF_DEBUG,
//      ("KSZ8851SNL_LongTransmitInit: Memory available >  txmir =%hu  reqSize = %hu \n", txmir, reqSize));
  /* Enable TXQ write access */
  data = ksz8851_reg_read(interface, REG_RXQ_CMD) | RXQ_START_DMA;
  ksz8851_reg_write(interface, REG_RXQ_CMD, data);

  /* Write frame ID, control word and byte count */
  outbuf[0] = (interface->frameId++ & FRAME_ID_MASK);
  outbuf[1] = 0x80; /*  TX_INT_on_COMPLETION */
  outbuf[2] = length & LSB_MASK;
  outbuf[3] = length >> MSB_POS;

  /* Start the SPI Transfer and send frame header */
  ksz8851_fifo_write(interface, outbuf, 4);
//  KSZ8851SNL_SPI_WriteFifoBegin();
//  KSZ8851SNL_SPI_WriteFifo(4, outbuf);

  return true;
}

void ksz8851_Transmit(struct KSZ8851_INTERFACE *interface, uint16_t length, uint8_t *buffer)
{
//  EFM_ASSERT(buffer != NULL);
  ksz8851_fifo_write(interface, buffer, length);
}

/***************************************************************************/
void ksz8851_TransmitEnd(struct KSZ8851_INTERFACE *interface, uint16_t length)
{
  uint16_t data;
  uint16_t padding;
  uint8_t dummy[4] =
  { 0x00 };

  /* Padding packet to 4 byte boundary */
  if (length % 4)
  {
    padding = 4 - (length % 4);
    ksz8851_fifo_write(interface, dummy, padding);
  }

  /* Stop the SPI Transfer */
//  KSZ8851SNL_SPI_WriteFifoEnd();
  /* Disable TXQ write access */
  data = ksz8851_reg_read(interface, RXQCR) & (~RXQ_START_DMA);
  ksz8851_reg_write(interface, RXQCR, data);

  /* Start TXQ Manual enqueue */
  data = ksz8851_reg_read(interface, TXQCR) | TXQ_ENQUEUE;
  ksz8851_reg_write(interface, TXQCR, data);
}

void ksz8851_ReleaseIncosistentFrame(struct KSZ8851_INTERFACE *interface)
{
//  uint16_t data;
  /* Issue the Release error frame command */
//  data = ksz8851_reg_read(interface, REG_RXQ_CMD); /* RXQCR */
//  data |= RXQ_RELEASE_ERROR_FRAME;
//  ksz8851_reg_write(interface, REG_RXQ_CMD, data); /* RXQCR */
  ksz8851_reg_setbits(interface, REG_RXQ_CMD, RXQ_RELEASE_ERROR_FRAME);  /* RXQCR */
  /* Wait for PHY to clear the command/flag */
  while (ksz8851_reg_read(interface, REG_RXQ_CMD) & RXQ_RELEASE_ERROR_FRAME)
  {
    ;
  }
}

// Jack, KSZ8851SNL Transmit Steps
/***************************************************************************//**
 * @brief Performs the actual transmit of a raw frame over the network.
 *
 * @param pTXLength
 *     the length of the transmitted frame
 * @param pTXData
 *     the data of the transmitted frame
 *****************************************************************************/
void ksz8851_Send(struct KSZ8851_INTERFACE *interface, uint8_t *pTXData, uint16_t pTXLength)
{
//  EFM_ASSERT(pTXData != NULL);
  // Page 22
  uint16_t data;
  uint16_t txmir;
  uint16_t txPacketLength;
  uint8_t outbuf[4096];

  /* Step 1 */
  /* Check if TXQ has enough memory for the transmission of the package */
  /* Read value from TXMIR to check if QMU TXQ has enough amount of memory
   * for the Ethernet packet data plus 4-byte frame header, plus 4-byte for
   * DWORD alignment. Compare the read value with (txPacketLength+4+4),
   * if less than (txPacketLength+4+4), Exit. */

  data = ksz8851_reg_read(interface, REG_TX_MEM_INFO); /* TXMIR */
  txmir = data & TX_MEM_AVAIL_MASK;     /* 0x1FFF */
  /* plus 4-byte frame header, plus 4-byte for DWORD alignment: EXTRA_SIZE = 8 bytes */
  txPacketLength = pTXLength + EXTRA_SIZE;
  if (txmir < txPacketLength)
  {
    // QMU TXQ does not have enough space
//    dmc_puts(TERMINAL_YELLOW);
//    dmc_puts("Wait until QMU TXQ space is available... ");

//    KSZ8851SNL_ExceptionHandler(INFO, "I will wait until mem is available\n");

    /* Enable TX memory available monitor */
    ksz8851_reg_write(interface, REG_TX_TOTAL_FRAME_SIZE, txPacketLength); /* TXNTFSR */
//    data = ksz8851_reg_read(interface, REG_TXQ_CMD);         /* TXQCR */
//    data |= TXQ_MEM_AVAILABLE_INT;
//    ksz8851_reg_write(interface, REG_TXQ_CMD, data);         /* TXQCR */
    ksz8851_reg_setbits(interface, REG_TXQ_CMD, TXQ_MEM_AVAILABLE_INT);  /* TXQCR */

    /* Wait until enough space is available */
    while (1)
    {
      data = ksz8851_reg_read(interface, REG_INT_STATUS);  /* ISR */
      if (data & INT_TX_SPACE)
      {
//        dmc_puts(".");
        break;
      }
    }
//    KSZ8851SNL_ExceptionHandler(INFO, "Done\n");
//    dmc_puts("Done\n");
//    dmc_puts(TERMINAL_DEFAULT);

    /* Clear flag */
    // Acknowledge (clear) INT_TX_SPACE Interrupt bit.
    // Note we have to set the bit in ISR to clear it!
//    data = ksz8851_reg_read(interface, REG_INT_STATUS);  /* ISR */
//    data |= INT_TX_SPACE;
//    ksz8851_reg_write(interface, REG_INT_STATUS, data);  /* ISR */
    ksz8851_reg_setbits(interface, REG_INT_STATUS, INT_TX_SPACE);  /* ISR */
  }

  /* Step 2 */
  /* Disable all interrupts on KSZ8851SNL */
  ksz8851_IntDisable(interface);
//  ksz8851_reg_write(interface, REG_INT_ENABLE, NO_INT);  /* IER */

  /* Step 3 */
  /* Enable TXQ write access */
//  data = ksz8851_reg_read(interface, REG_RXQ_CMD); /* RXQCR */
//  data |= RXQ_START;            /* bit 3 or 0x0008 */
//  ksz8851_reg_write(interface, REG_RXQ_CMD, data); /* RXQCR */
  ksz8851_reg_setbits(interface, REG_RXQ_CMD, RXQ_START);  /* RXQCR */

  // For debugging probe
//  HAL_GPIO_WritePin(GPIOF, PF13_Test_Pin, GPIO_PIN_SET);
//  HAL_GPIO_WritePin(GPIOF, PF13_Test_Pin, GPIO_PIN_SET);
//  HAL_GPIO_WritePin(GPIOF, PF13_Test_Pin, GPIO_PIN_SET);
//  HAL_GPIO_WritePin(GPIOF, PF13_Test_Pin, GPIO_PIN_RESET);

  //  dmc_puts("Perform blocking SPI transfer\n");

//  // 5 bytes
//  outbuf[0] = FIFO_WRITE;
//  outbuf[1] = (interface->frameId++ & FRAME_ID_MASK);
//  outbuf[2] = 0;
//  outbuf[3] = pTXLength & LSB_MASK;
//  outbuf[4] = pTXLength >> MSB_POS;
//  /* Perform blocking SPI transfer. */
//  (void) HAL_SPI_Transmit(interface->hspi, (uint8_t*) outbuf, 5, 2000);

//  // 4 bytes
//  outbuf[0] = (interface->frameId++ & FRAME_ID_MASK) | TX_INT_on_COMPLETION >> MSB_POS;
//  outbuf[1] = 0;
//  outbuf[2] = pTXLength & LSB_MASK;
//  outbuf[3] = pTXLength >> MSB_POS;
//  for (uint16_t i = 0; i < pTXLength; i++)
//  {
//    pTXData2[i + 4] = pTXData[i];
//  }
//  pTXLength += 4;

  // Step 6
  // Write TXIC to the "control word" of the frame header. (0x8000)
  // Number of SCLK for register access as following:
  // Write frame to TXQ: CMD(8bits) + Frame Header(32bits) + Frame Data(8bits*N)
  // Read frame from RXQ: CMD(8bits) + Dummy(32bits) + Frame Status(32bits) + Frame Data(8bits*N)
//  pTXData[0] = FIFO_WRITE;
//  pTXData[1] = (interface->frameId++ & FRAME_ID_MASK);
////  pTXData[1] = (interface->frameId++ & FRAME_ID_MASK) | TX_INT_on_COMPLETION >> MSB_POS;
//  pTXData[2] = 0;
//  // Step 7
//  // Write txPacketLength to the "byte count" of the frame header.
//  uint16_t TXLen = pTXLength - 5;
//  pTXData[3] = TXLen & LSB_MASK;
//  pTXData[4] = TXLen >> MSB_POS;


  // Write frame data pointer by pTxData to the QMU TXQ in BYTE until
  // finished the full packet length (txPacketLength) in DWORD alignment "bytesToWrite"
  /* Round to DWORD boundary, for example, the driver has to write up
   * to 68 bytes if transmit frame size is 65 bytes. */
//  uint16_t bytesToWrite = 4 * ((TXLen + 3) >> 2) + 5;
//  int lengthInDWord=((pTXLength+3)>>2);
//  int bytesToWrite=(lengthInDWord *4);
//  dmc_putintcr(lengthInByte);

  // https://community.st.com/s/article/FAQ-DMA-is-not-working-on-STM32H7-devices
//  __DSB();

  ksz8851_fifo_write(interface, pTXData, pTXLength);




//  uint16_t bytesToWrite = 4 * ((pTXLength + 3) >> 2);
//
//  // DMA is not working on STM32H7 devices
//  // https://community.st.com/s/article/FAQ-DMA-is-not-working-on-STM32H7-devices
//  /* ATTENTION: pad bytes are the bytes beyond buffer length (can be any rubbish value) */
//  /* Perform non-blocking DMA SPI transfer. Discard function returned value! TODO: handle it? */
////  (void) HAL_SPI_Transmit_DMA(interface->spi, (uint8_t*) pTXData, lengthInByte);
//  /* an DMA1_Stream4_IRQHandler interrupt will occur */
//
//  /* Pull the CS pin low */
//  HAL_GPIO_WritePin((GPIO_TypeDef*) interface->cs_port, interface->cs_pin, GPIO_PIN_RESET);
//
//uint8_t TxHeader[5];
//// Step 6
//// Write TXIC to the "control word" of the frame header. (0x8000)
//// Number of SCLK for register access as following:
//// Write frame to TXQ: CMD(8bits) + Frame Header(32bits) + Frame Data(8bits*N)
//// Read frame from RXQ: CMD(8bits) + Dummy(32bits) + Frame Status(32bits) + Frame Data(8bits*N)
//TxHeader[0] = FIFO_WRITE;
//TxHeader[1] = (interface->frameId++ & FRAME_ID_MASK);
////  pTXData[1] = (frameId++ & FRAME_ID_MASK) | TX_INT_on_COMPLETION >> MSB_POS;
//TxHeader[2] = 0;
//// Step 7
//// Write txPacketLength to the "byte count" of the frame header.
//TxHeader[3] = pTXLength & LSB_MASK;
//TxHeader[4] = pTXLength >> MSB_POS;
//
//  /* Perform blocking SPI transfer. */
//  (void) HAL_SPI_TransmitReceive(interface->hspi, (uint8_t*) TxHeader, (uint8_t*) outbuf, 5, 2000);
//
//#if (KSZ8851_USE_TX_DMA == 0)
////  dmc_puts("Perform blocking SPI transfer\n");
//
//  /* Perform blocking SPI transfer. */
//  (void) HAL_SPI_TransmitReceive(interface->hspi, (uint8_t*) pTXData, (uint8_t*) outbuf, bytesToWrite, 2000);
////  dmc_puts("Done\n");
//#else
//  clr_dma_tx_ended(interface);
//  dmc_puts("Perform non-blocking DMA SPI TX transfer\n");
//  /* Perform non-blocking DMA SPI transfer. Discard function returned value! TODO: handle it? */
//  (void) HAL_SPI_TransmitReceive_DMA(interface->hspi, (uint8_t*) pTXData, (uint8_t*) outbuf, bytesToWrite);
//  /* For SPI1_TX an DMA1_Stream0_IRQHandler interrupt will occur */
//  /* For SPI1_RX an DMA1_Stream1_IRQHandler interrupt will occur */
//  /* For SPI4_RX an DMA1_Stream2_IRQHandler interrupt will occur */
//  /* For SPI4_TX an DMA1_Stream3_IRQHandler interrupt will occur */
//  dmc_puts("Done\n");
//  wait_dma_tx_ended(interface);
//  dmc_puts("Done\n");
//#endif
//
//  /* Terminate the operation by raising the CS pin */
//  HAL_GPIO_WritePin((GPIO_TypeDef*) interface->cs_port, interface->cs_pin, GPIO_PIN_SET);




  /* Start the SPI Transfer */
//  ETHSPI_StartWriteFIFO();
  /* Send the frame header info */
//  ETHSPI_WriteFifoContinue(4, outbuf);

  /* Send the actual data */
//  ETHSPI_WriteFifoContinue(pTXLength, pTXData);

  /* Send dummy bytes to align data to DWORD */
//  ETHSPI_WriteFifoContinue(KSZ8851SNL_DwordAllignDiff(pTXLength), pTXData);

  /* Stop the SPI Transfer */
//  ETHSPI_StopFIFO();

  /* Step 12 */
  /* Disable TXQ write access */
//  data = ksz8851_reg_read(interface, REG_RXQ_CMD); /* RXQCR */
//  data &= ~RXQ_START;           /* bit 3 or 0x0008 */
//  ksz8851_reg_write(interface, REG_RXQ_CMD, data); /* RXQCR */
  ksz8851_reg_clrbits(interface, REG_RXQ_CMD, RXQ_START);  /* RXQCR */

  /* Step 12.1 */
  /* Start TXQ Manual Enqueue */
//  data = ksz8851_reg_read(interface, REG_TXQ_CMD); /* RXQCR */
//  data |= TXQ_ENQUEUE;          /* bit 0 or 0x0001 */
//  ksz8851_reg_write(interface, REG_TXQ_CMD, data); /* RXQCR */
  ksz8851_reg_setbits(interface, REG_TXQ_CMD, TXQ_ENQUEUE);  /* RXQCR */

  /* Wait until transmit command clears */
  while (1)
  {
    data = ksz8851_reg_read(interface, REG_TXQ_CMD); /* RXQCR */
    if (!(data & TXQ_ENQUEUE))        /* bit 0 or 0x0001 */
    {
      break;
    }
  }

  /* Step 13 */
  /* Enable interrupts */
//  ksz8851_reg_write(interface, REG_INT_ENABLE, INT_MASK_EXAMPLE);
  /* Enable interrupts */
  ksz8851_IntEnable(interface);
}

// Jack, KSZ8851SNL Receive Steps
/***************************************************************************//**
 * @brief Performs the actual receive of a raw frame over the network.
 *
 * @param pRXLength
 *     the length of the received frame
 * @param pRXData
 *     the data of the received frame
 *
 * @return
 *     received packet length, 0 in case of failure
 *****************************************************************************/
uint16_t ksz8851_Receive(struct KSZ8851_INTERFACE *interface, uint8_t *pRXData, uint16_t pRXLength)
{
  uint16_t data;
  uint16_t rxFrameCount;
  uint16_t rxStatus;
  uint16_t rxPacketLength;
//  uint16_t frameLength;
  uint16_t bytesToRead;
  uint8_t outbuf[4096];

//    dmc_puts("ksz8851_Receive\n");
//  FR_HEADER_INFO_t rxFrameHeader[MAX_FRAMES_IN_RXQ];

//  EFM_ASSERT(buffer != NULL);
//  EFM_ASSERT(pRXData != NULL);
//  EFM_ASSERT(pRXLength != NULL);

  // Step 1
  /* RX step1: read and discard interrupt status for INT_RX flag. */
  // Read value from ISR to check if RXIS ‘Receive Interrupt’ is set. If not set, Exit.
  data = ksz8851_reg_read(interface, REG_INT_STATUS);  /* ISR */

  // Jack 27-03-2019 Reads 0x0000!!!
//  dmc_puthex4cr(data);

  if (!(data & IRQ_RXI))  /* INT_RX_DONE */
  {
    return 0;
  }
//  dmc_puts("OK\n");
//  dmc_puts(TERMINAL_CYAN);
//  dmc_puts("Handle IRQ_RXI\n");
//  dmc_puts(TERMINAL_DEFAULT);

  // Step 2
  // Disable all the device interrupts generation.
  ksz8851_IntDisable(interface);

  // Step 3
  /* RX step3: clear INT_RX flag. */
  // Acknowledge (clear) RXIS Receive Interrupt bit.
  // Note we have to set the bit in ISR to clear it!
//  data = ksz8851_reg_read(interface, REG_INT_STATUS);  /* ISR */
//  data |= IRQ_RXI;              /* RXIS */
//  ksz8851_reg_write(interface, REG_INT_STATUS, data);  /* ISR */
  ksz8851_reg_setbits(interface, REG_INT_STATUS, IRQ_RXI);

//  intStatus = ksz8851_reg_read(interface, REG_INT_STATUS); /* ISR */
//  dmc_puts("REG_INT_STATUS ");
//  dmc_puthex4cr(intStatus);
  // When receive interrupt occurred and software driver writes "1" to clear
  // the RX interrupt in ISR register;
  // the KSZ8851 will update Receive Frame Counter (RXFCTR) Register for this interrupt.

  // Step4
  /* RX step4-5: check for received frames. */
  // Read current total amount of received frame count from RXFCTR, and save in ‘rxFrameCount’.
  /* Read the frame count and threshold register */
  data = ksz8851_reg_read(interface, REG_RX_FRAME_CNT_THRES);  /* RXFCTR */
  /* Extract the actual number of frames from RX_FRAME_THRES_REG*/
  rxFrameCount = (data >> MSB_POS) & 0x00ff;
  // When software driver reads back Receive Frame Count (RXFCTR) Register;
  // the KSZ8851 will update both Receive Frame Header Status
  // and Byte Count Registers (RXFHSR/RXFHBCR).

  if (rxFrameCount == 0)
  {
    /* Enable interrupts */
    ksz8851_IntEnable(interface);
    return 0;
  }

//  dmc_puts(TERMINAL_MAGENTA);
//  dmc_puts("rxFrameCount: ");
//  dmc_putintcr(rxFrameCount);
//  dmc_puts(TERMINAL_DEFAULT);

  /* Now rxFrameCount != 0 */
  /* Don't break Micrel state machine, wait for a free descriptor first! */
  // Step 5
  // Repeatedly reading all frames from RXQ.
  // If rxFrameCount <= 0, goto step 24
  while (rxFrameCount > 0)
  {
    // Step 23
    // Jack: we do this first, so we can use the "continue" statement
    // to skip the rest of the iteration, and continue to next frame
    rxFrameCount--;

    // When software driver reads back both Receive Frame Header Status and Byte Count Registers (RXFHSR/RXFHBCR);
    // the KSZ8851 will update next receive frame header status and byte count registers (RXFHSR/RXFHBCR).

    // Step 6
    /* RX step6: get RX packet status. */
    // Read received frame status from RXFHSR to check if this is a good frame.
    // This register (read only) indicates the current received frame header status information.
    /* Read the received frame status */
    rxStatus = ksz8851_reg_read(interface, REG_RX_FHR_STATUS);   /* RXFHSR */
//    dmc_puts("rxStatus: ");
//    dmc_puthex4cr(rxStatus);

    // Step 7
    // Read received frame byte size from RXFHBCR to get this received frame length
    // (4-byte CRC, and extra 2-byte due to "IP Header Two-Byte Offset Enable" are included),
    // and store into rxPacketLength variable.
//    data = ksz8851_reg_read(interface, REG_RX_FHR_BYTE_CNT); /* RXFHBCR */
//    rxPacketLength = data & RX_BYTE_CNT_MASK;   /* 0x0FFF */
////    rxPacketLength -= 11; // 11 preceeding bytes
//    dmc_puts("rxPacketLength: ");
//    dmc_putintcr(rxPacketLength);

    /* Check the consistency of the frame */
    // if rxStatus’s bit_15 is 0, or
    // if rxStatus’s bit_0, bit_1, bit_2, bit_4, bit_10, bit_11, bit_12, bit_13 are 1,
    // received an error frame, goto step 8,
    // else received a good frame, goto step 9.
    // if rxPacketLength <= 0, goto step 8;
    // else goto step 9;
    // CHECKSUM_VALID_FRAME_MASK = 0011 1100 0001 0111 = 0x3C17 (bits 13,12,11,10,4,2,1,0)
    if ((!(rxStatus & RX_VALID)) || (rxStatus & RX_ERRORS))
    {
      // RX packet error!
      /* Issue the Release error frame command */
      ksz8851_ReleaseIncosistentFrame(interface);
      data = ksz8851_reg_read(interface, REG_RX_FRAME_CNT_THRES); /* RXFCTR */
      /* Extract the actual number of frames from RX_FRAME_THRES_REG*/
      rxFrameCount = (data >> MSB_POS) & 0x00ff;
      // When software driver reads back Receive Frame Count (RXFCTR) Register;
      // the KSZ8851 will update both Receive Frame Header Status
      // and Byte Count Registers (RXFHSR/RXFHBCR).

      if (rxFrameCount == 0)
      {
        /* Enable interrupts */
        ksz8851_ReleaseIncosistentFrame(interface);
        ksz8851_IntEnable(interface);
        return 0;
      }

      /* continue to next frame */
       continue;
    }
    else
    {
//      dmc_puts(TERMINAL_MAGENTA);
//      dmc_puts("rxFrameCount: ");
//      dmc_putintcr(rxFrameCount);
//      dmc_puts(TERMINAL_DEFAULT);

      // Step 7
      /* RX step7: read frame length. */
      // Read received frame byte size from RXFHBCR to get this received frame length
      // (4-byte CRC, and extra 2-byte due to ‘IP Header Two-Byte Offset Enable’are included),
      // and store into rxPacketLength variable.
      /* Read the byte size of the received frame */
      /* Get current frame byte size */
      /* Frame length includes 4 byte CRC and extra 2 byte IP Header */
      rxPacketLength = ksz8851_reg_read(interface, REG_RX_FHR_BYTE_CNT) & RX_BYTE_CNT_MASK;  /* RXFHBCR */
//      dmc_puts("rxPacketLength: ");
//      dmc_putintcr(rxPacketLength);

      /* RX step8: Drop packet if len is invalid or no descriptor available. */
      if(rxPacketLength == 0)
      {
        // RX bad len!
        ksz8851_ReleaseIncosistentFrame(interface);
//        ksz8851_IntEnable(interface);
//        return 0;
        continue;
      }

      // Step 17
      // Read frame data to system memory pointer by pRxData from the QMU RXQ in BYTE until
      // finished the full packet length (rxPacketLength) in DWORD alignment ‘lengthInByte.
      /* Round to DWORD boundary
       * For example, the driver has to read up to 68 bytes if received frame size is 65 bytes. */
      // Jack: read a bit more because of missing bytes
      // Read frame from RXQ: CMD(8bits) + Dummy(32bits) + Frame Status(32bits) + Frame Data(8bits*N)
      bytesToRead = 4 * ((rxPacketLength + 3) >> 2);
//      dmc_puts("bytesToRead: ");
//      dmc_putintcr(bytesToRead);

      if ((bytesToRead > pRXLength) || (rxPacketLength <= 4))
      {
        /* Issue the Release error frame command */
        ksz8851_ReleaseIncosistentFrame(interface);
//        ksz8851_IntEnable(interface);
//        return 0;
//        /* continue to next frame */
        continue;
      }

      // Step 9
      /* RX step9: reset RX frame pointer. */
      // Reset QMU RXQ frame pointer to zero with auto increment.
//      ksz8851_reg_write(interface, REG_RX_ADDR_PTR, ADDR_PTR_AUTO_INC);
      /* Set QMU RXQ frame pointer to start of packet data. Note
       * that we skip the status word and byte count since we
       * already know this from RXFHSR and RXFHBCR.
       */
      // This pointer value must reset to 0x000 before each DMA operation from
      // the host CPU to read RXQ frame buffer.
      // When this bit is set, the RXQ Address register increments automatically
      // on accesses to the data register. The increment is by one for every byte
      // access, by two for every word access, and by four for every double word
      // access.
      // When this bit is reset, the RX frame data pointer is manually controlled
      // by user to access the RX frame location.
//      ksz8851_reg_write(interface, REG_RX_ADDR_PTR, ADDR_PTR_AUTO_INC | 0x0004); /* RXFDPR */
//      ksz8851_reg_write(interface, REG_RX_ADDR_PTR, ADDR_PTR_AUTO_INC | 0x0000); /* RXFDPR */
      ksz8851_reg_clrbits(interface, REG_RX_ADDR_PTR, ADDR_PTR_MASK);

      // Step 10
      /* RX step10: start RXQ read access. */
      // Start QMU DMA transfer operation to read frame data from the RXQ to host CPU.
      // When this bit is written as 1, the KSZ8851SNL allows a DMA operation
      // from the host CPU to access either read RXQ frame buffer or write TXQ
      // frame buffer with SPI command operation for RXQ/TXQ FIFO read/
      // write. All registers access are disabled except this register during this
      // DMA operation.
      // This bit must be set to 0 when DMA operation is finished in order to
      // access the rest of registers.
      /* Set bit 3 of RXQCR to start QMU DMA transfer operation */
      /* Must not access other registers once starting QMU DMA transfer operation */
//      data = ksz8851_reg_read(interface, REG_RXQ_CMD);
//      data |= RXQ_START;
//      ksz8851_reg_write(interface, REG_RXQ_CMD, data);
//      ksz8851_reg_setbits(interface, REG_RXQ_CMD, RXQ_START);  /* RXQCR */
      ksz8851_reg_setbits(interface, REG_RXQ_CMD, RXQ_START | RXQ_AUTO_DEQUEUE);

//      /* Read the whole ethernet frame */
//      ksz8851_fifo_read(interface, pRXData, bytesToRead);

      // Step 13
      // Must read out dummy 4-byte.
//      dmc_puts("Step 13\n");
//      uint8_t Dummy[4];
//      (void) HAL_SPI_Receive(interface->hspi, (uint8_t*) Dummy, 4, 2000);

//      // Step 14
//      // Read out 2-byte ‘Status Word’ of frame header from the QMU RXQ.
//      dmc_puts("Step 14\n");
//      uint8_t StatusWord[2];
//      (void) HAL_SPI_Receive(interface->hspi, (uint8_t*) StatusWord, 2, 2000);
//
//      // Step 15
//      // Read out 2-byte ‘Byte Count’ of frame header from the QMU RXQ.
//      dmc_puts("Step 15\n");
//      uint8_t ByteCount[2];
//      (void) HAL_SPI_Receive(interface->hspi, (uint8_t*) ByteCount, 2, 2000);
//      dmc_puts("ByteCount: ");
//      dmc_puthex2(ByteCount[1]);
//      dmc_puthex2cr(ByteCount[0]);

      // Step 18
      // Read 1-byte of frame data to system memory pointer by pRxData from
      // the QMU RXQ. Increase pRxData pointer by 1.
      /* Perform blocking SPI transfer. */
//      dmc_puts("Perform blocking SPI transfer.\n");
//      (void) HAL_SPI_Receive(interface->hspi, (uint8_t*) pRXData, bytesToRead, 2000);
      // Number of SCLK for register access as following:
      // Write frame to TXQ: CMD(8bits) + Frame Header(32bits) + Frame Data(8bits*N)
      // Read frame from RXQ: CMD(8bits) + Dummy(32bits) + Frame Status(32bits) + Frame Data(8bits*N)
      /* Write RXQ command phase */
      memset(outbuf, 0, bytesToRead);
      outbuf[0] = FIFO_READ;
      /* Dummy 4 byte, status word 2 byte, byte count 2 byte, IP header 2 byte */
      /* Buffer allocation = number of bytes to receive */
//      bytesToRead += 12;

//      bytesToRead += 8;

//dmc_putintcr(bytesToRead);
//      memset(pRXData, 0x55, bytesToRead);

//      ksz8851_fifo_read(interface, pRXData, rxPacketLength);
      rxPacketLength = ksz8851_fifo_read(interface, pRXData, bytesToRead);





//      /* Pull the CS pin low */
//      HAL_GPIO_WritePin((GPIO_TypeDef*) interface->cs_port, interface->cs_pin, GPIO_PIN_RESET);
//
//#if (KSZ8851_USE_RX_DMA == 0)
//      /* Perform blocking SPI transfer. */
//      (void) HAL_SPI_TransmitReceive(interface->hspi, (uint8_t*) outbuf, (uint8_t*) outbuf, 15, 2000);
//      bytesToRead -= 15;
//
////      dmc_puts("Perform blocking SPI transfer\n");
//      (void) HAL_SPI_Receive(interface->hspi, (uint8_t*) pRXData, bytesToRead, 2000);
////      dmc_puts("Done\n");
//#else
//      /* Perform blocking SPI transfer. */
//      (void) HAL_SPI_TransmitReceive(interface->hspi, (uint8_t*) outbuf, (uint8_t*) outbuf, 15, 2000);
//      bytesToRead -= 15;
//
//      clr_dma_rx_ended(interface);
//      dmc_puts("Perform non-blocking DMA SPI RX transfer\n");
//      /* Perform non-blocking DMA SPI transfer. Discard function returned value! TODO: handle it? */
////      (void) HAL_SPI_TransmitReceive_DMA(interface->hspi, (uint8_t*) outbuf, (uint8_t*) pRXData, bytesToRead);
//      (void) HAL_SPI_Receive_DMA(interface->hspi, (uint8_t*) pRXData, bytesToRead);
//      /* For SPI1_TX an DMA1_Stream0_IRQHandler interrupt will occur */
//      /* For SPI1_RX an DMA1_Stream1_IRQHandler interrupt will occur */
//      /* For SPI4_RX an DMA1_Stream2_IRQHandler interrupt will occur */
//      /* For SPI4_TX an DMA1_Stream3_IRQHandler interrupt will occur */
//      dmc_puts("Done\n");
//      wait_dma_rx_ended(interface);
//      dmc_puts("Done\n");
//#endif
//
//      /* Terminate the operation by raising the CS pin */
//      HAL_GPIO_WritePin((GPIO_TypeDef*) interface->cs_port, interface->cs_pin, GPIO_PIN_SET);




      // Step 21
      // Stop QMU DMA transfer operation.
//      data = ksz8851_reg_read(interface, REG_RXQ_CMD);
//      data &= ~RXQ_START;
//      ksz8851_reg_write(interface, REG_RXQ_CMD, data);
      ksz8851_reg_clrbits(interface, REG_RXQ_CMD, RXQ_START);

      // Step 22
      // Pass this received frame to the upper layer protocol stack.
      // Jack: may print it if it makes sense...
      // 192.168.25.xxx
//      uint8_t found = 0;
////      for (uint8_t i = 0; i < 30; i++)
////      {
////        if ((pRXData[i] == 0xC0) && (pRXData[i+1] == 0xA8) && (pRXData[i+2] == 0xA8))
////        {
////          found = 1;
////        }
////      }
//
//      uint16_t offset1 = 0;
//      uint16_t offset2 = 0;
//
//      for (uint16_t i = 30; i < 64; i++)
//      {
//        if ((pRXData[i] == 0xC0) && (pRXData[i+1] == 0xA8) && (pRXData[i+2] == 0x19))
//        {
//          if (offset1 == 0)
//          {
//            offset1 = i;
//          }
//          else
//          {
//            offset2 = i;
//            found = 1;
//            break;
//          }
//        }
//      }
//
////      found = 1;
//      if (found)
//      {
////        dmc_puthex2(pRXData[offset++]);
////        dmc_putc(':');
////        dmc_puthex2(pRXData[offset++]);
////        dmc_putc(':');
////        dmc_puthex2(pRXData[offset++]);
////        dmc_putc(':');
////        dmc_puthex2(pRXData[offset++]);
////        dmc_putc(':');
////        dmc_puthex2(pRXData[offset++]);
////        dmc_putc(':');
////        dmc_puthex2(pRXData[offset++]);
////        dmc_putc(' ');
//        dmc_putint(pRXData[offset1++]);
//        dmc_putc('.');
//        dmc_putint(pRXData[offset1++]);
//        dmc_putc('.');
//        dmc_putint(pRXData[offset1++]);
//        dmc_putc('.');
//        uint8_t ipa4 = pRXData[offset1++];
//        dmc_putint(ipa4);
//        dmc_putc('\n');
//
////        dmc_puthex2(pRXData[offset++]);
////        dmc_putc(':');
////        dmc_puthex2(pRXData[offset++]);
////        dmc_putc(':');
////        dmc_puthex2(pRXData[offset++]);
////        dmc_putc(':');
////        dmc_puthex2(pRXData[offset++]);
////        dmc_putc(':');
////        dmc_puthex2(pRXData[offset++]);
////        dmc_putc(':');
////        dmc_puthex2(pRXData[offset++]);
////        dmc_putc(' ');
//        dmc_putint(pRXData[offset2++]);
//        dmc_putc('.');
//        dmc_putint(pRXData[offset2++]);
//        dmc_putc('.');
//        dmc_putint(pRXData[offset2++]);
//        dmc_putc('.');
//        uint8_t ipb4 = pRXData[offset2++];
//        dmc_putint(ipb4);
//        dmc_putc('\n');
//
//        uint8_t ip4 = 238;
//        if ((ipa4 == ip4) || (ipb4 == ip4))
//        {
//          dmc_puts(TERMINAL_YELLOW);
//        }
//        else
//        {
//          dmc_puts(TERMINAL_CYAN);
//        }
//        for (uint16_t i = 0; i < bytesToRead; i++)
//        {
//          dmc_puthex2(pRXData[i]);
//          dmc_putc(' ');
//        }
//        dmc_putc('\n');
//        uint8_t pcr = 0;
//        for (uint16_t i = 0; i < bytesToRead; i++)
//        {
//          if ((pRXData[i] >= 0x20) && (pRXData[i] < 0x7F))
//          {
//            dmc_putc(pRXData[i]);
//            pcr = 1;
//          }
//        }
//        if (pcr)
//        {
//          dmc_putc('\n');
//        }
//        dmc_puts(TERMINAL_DEFAULT);
//      }
      break;
    }
  }


//  data = ksz8851_reg_read(interface, REG_INT_ENABLE);  /* ISR */
//  dmc_puts("REG_INT_ENABLE ");
//  dmc_puthex4cr(data);

  // Step 24
  /* Enable interrupts */
  ksz8851_IntEnable(interface);

  // After enabling interrupts, an interrupt is generated immediately

//  data = ksz8851_reg_read(interface, REG_INT_ENABLE);  /* ISR */
//  dmc_puts("REG_INT_ENABLE ");
//  dmc_puthex4cr(data);

//  return bytesToRead;

  /* Return frame length without CRC and 2 extra bytes */
//  rxPacketLength -= 6;
//  rxPacketLength -= 11; // Strip 11 characters
//  frameLength = rxPacketLength - 2;
//  frameLength = rxPacketLength + 2;
  return rxPacketLength;




//  /* Read the frame count and threshold register */
//  rxftr = ksz8851_reg_read(interface, REG_RX_FRAME_CNT_THRES); /* RXFCTFC */
//  /* Extract the actual number of frames from RX_FRAME_THRES_REG*/
//  rxFrameCount = rxftr >> MSB_POS;
//
//  while (rxFrameCount > 0)
//  {
//    rxFrameCount--;
//    /* Read the received frame status */
//    rxStatus = ksz8851_reg_read(interface, REG_RX_FHR_STATUS);
//
//    /* Check the consistency of the frame */
//    if ((!(rxStatus & VALID_FRAME_MASK)) || (rxStatus & CHECKSUM_VALID_FRAME_MASK))
//    {
//      /* Issue the Release error frame command */
//      ksz8851_ReleaseIncosistentFrame(interface);
//      /* continue to next frame */
//      continue;
//    }
//    else
//    {
//      /* Read the byte size of the received frame */
//      rxPacketLength = ksz8851_reg_read(interface, RXFHBCR) & RX_BYTE_CNT_MASK;
//
//      /* round to dword boundary */
//      bytesToRead = 4 * ((rxPacketLength + 3) >> 2);
////      LWIP_DEBUGF(NETIF_DEBUG,
////          ("KSZ8851SNL_Receive: rxPacketLength=%u, bytesToRead=%u \n", rxPacketLength, bytesToRead));
//      if ((bytesToRead > pRXLength) || (rxPacketLength <= 4))
//      {
//        /* Issue the Release error frame command */
//        ksz8851_ReleaseIncosistentFrame(interface);
//        /* continue to next frame */
//        continue;
//      }
//
//      /* Set QMU RXQ frame pointer to start of packet data. Note
//       * that we skip the status word and byte count since we
//       * already know this from RXFHSR and RXFHBCR.
//       */
//      ksz8851_reg_write(interface, REG_RX_ADDR_PTR, 0x0004 | FD_PTR_AUTO_INC);
//
//      /* Start QMU DMA transfer */
//      data = ksz8851_reg_read(interface, REG_RXQ_CMD);
//      data |= RXQ_START_DMA;
//      ksz8851_reg_write(interface, REG_RXQ_CMD, data);
//
//      /* Read the whole ethernet frame */
//      ksz8851_fifo_read(interface, pRXData, bytesToRead);
//
//      /* Stop QMU DMA transfer */
//      data = ksz8851_reg_read(interface, REG_RXQ_CMD);
//      data &= ~RXQ_START_DMA;
//      ksz8851_reg_write(interface, REG_RXQ_CMD, data);
//
//      /* Enable interrupts */
//      data = INT_MASK_EXAMPLE;
//      ksz8851_reg_write(interface, REG_INT_ENABLE, data);
//
//      /* Return frame length without CRC */
//      frameLength = rxPacketLength - 4;
//      return frameLength;
//    }
//    /* Enable interrupts */
//    data = INT_MASK_EXAMPLE;
//    ksz8851_reg_write(interface, REG_INT_ENABLE, data);
//  }
//  return 0;
}

uint16_t ksz8851_PHYStatusGet(struct KSZ8851_INTERFACE *interface)
{
  return ksz8851_reg_read(interface, P1SR);
}

void ksz8851_SetDigitalLoopbackMode(struct KSZ8851_INTERFACE *interface)
{
  uint16_t data;
  /* Reset PHY. */
  data = PHY_RESET;
  ksz8851_reg_write(interface, REG_PHY_RESET, data);
  /* Disable Auto-negotiation.1. Reset PHY. */
  /* Set Speed to either 100Base-TX or 10Base-T. */
  /* Set Duplex to full-duplex. */
  /* Set PHY register 0.14 to Â‘1Â’ to enable Local Loop-back. */
  data = DIGITAL_LOOPBACK | FORCE_FULL_DUPLEX | FORCE_100;
  data &= ~AUTO_NEG;
  ksz8851_reg_write(interface, REG_PHY_CNTL, data);

//  KSZ8851SNL_ExceptionHandler(INFO, "Loopback mode initiated");
}

/**************************************************************************//**
 * @brief enables the chip interrupts
 *****************************************************************************/
void ksz8851_EnableInterrupts(struct KSZ8851_INTERFACE *interface)
{
  /* Enable interrupts */
  ksz8851_reg_write(interface, INT_ENABLE_REG, INT_MASK_EXAMPLE);
}

// https://github.com/EnergyMicro/kit_common/blob/master/drivers/ksz8851snl.c
/***************************************************************************//**
 * @brief Checks for any interrrupts and if found, clears their status
 *        and prepair for interrupt handler routines
 * @note  Programmer needs to re-enable the KSZ8851SNL interrupts after
 *        calling this function
 *
 * @return
 *     found interrupts
 *
 *****************************************************************************/
uint16_t ksz8851_CheckIrqStat(struct KSZ8851_INTERFACE *interface)
{
  uint16_t data, ISR_stat, found_INT;
  found_INT = 0;
  ISR_stat = ksz8851_reg_read(interface, REG_INT_STATUS);

  /* Disable interrupts on KSZ8851SNL */
  data = NO_INT;
  ksz8851_reg_write(interface, REG_INT_ENABLE, data);

  /* Resolve the RX completion interrupt */
  if (ISR_stat & INT_RX_DONE)
  {
    dmc_puts("INT_RX_DONE\n");
    /* Clear RX Interrupt flag */
    data = ksz8851_reg_read(interface, REG_INT_STATUS);
    data = INT_RX_DONE;
    ksz8851_reg_write(interface, REG_INT_STATUS, data);
    found_INT |= INT_RX_DONE;
  }
  /* Resolve the Link change interrupt */
  if (ISR_stat & INT_LINK_CHANGE)
  {
//    dmc_puts("INT_LINK_CHANGE\n");
    /* Clear Link change Interrupt flag */
    data = ksz8851_reg_read(interface, REG_INT_STATUS);
    data = INT_LINK_CHANGE;
    ksz8851_reg_write(interface, REG_INT_STATUS, data);
    found_INT |= INT_LINK_CHANGE;
  }
  /* Resolve the RX overrun interrupt */
  if (ISR_stat & INT_RX_OVERRUN)
  {
//    dmc_puts("INT_RX_OVERRUN\n");
    /* Clear RX overrun Interrupt flag */
    data = ksz8851_reg_read(interface, REG_INT_STATUS);
    data = INT_RX_OVERRUN;
    ksz8851_reg_write(interface, REG_INT_STATUS, data);
    found_INT |= INT_RX_OVERRUN;
  }
  /* Resolve the TX stopped interrupt */
  if (ISR_stat & INT_TX_STOPPED)
  {
//    dmc_puts("INT_TX_STOPPED\n");
    /* Clear TX stopped Interrupt flag */
    data = ksz8851_reg_read(interface, REG_INT_STATUS);
    data = INT_TX_STOPPED;
    ksz8851_reg_write(interface, REG_INT_STATUS, data);
    found_INT |= INT_TX_STOPPED;
  }
  /* Resolve the RX stopped interrupt */
  if (ISR_stat & INT_RX_STOPPED)
  {
//    dmc_puts("INT_RX_STOPPED\n");
    /* Clear RX stopped Interrupt flag */
    data = ksz8851_reg_read(interface, REG_INT_STATUS);
    data = INT_RX_STOPPED;
    ksz8851_reg_write(interface, REG_INT_STATUS, data);
    found_INT |= INT_RX_STOPPED;
  }
  /* Resolve the RX of a WakeOnLan frame interrupt */
  if (ISR_stat & INT_RX_WOL_FRAME)
  {
//    dmc_puts("INT_RX_WOL_FRAME\n");
    /* Clear RX of a WakeOnLan Interrupt flag */
    data = ksz8851_reg_read(interface, REG_INT_STATUS);
    data = INT_RX_WOL_FRAME;
    ksz8851_reg_write(interface, REG_INT_STATUS, data);
    found_INT |= INT_RX_WOL_FRAME;
  }
  /* Resolve the RX of a magic frame interrupt */
  if (ISR_stat & INT_MAGIC)
  {
//    dmc_puts("INT_MAGIC\n");
    /* Clear RX of a magic frame Interrupt flag */
    data = ksz8851_reg_read(interface, REG_INT_STATUS);
    data = INT_MAGIC;
    ksz8851_reg_write(interface, REG_INT_STATUS, data);
    found_INT |= INT_MAGIC;
  }
  /* Resolve the RX of a LINKUP interrupt */
  if (ISR_stat & INT_LINKUP)
  {
//    dmc_puts("INT_LINKUP\n");
    /* Clear RX of a LINKUP Interrupt flag */
    data = ksz8851_reg_read(interface, REG_INT_STATUS);
    data = INT_LINKUP;
    ksz8851_reg_write(interface, REG_INT_STATUS, data);
    found_INT |= INT_LINKUP;
  }
  /* Resolve the RX of a Energy interrupt */
  if (ISR_stat & INT_ENERGY)
  {
//    dmc_puts("INT_ENERGY\n");
    /* Clear RX of a Energy Interrupt flag */
    data = ksz8851_reg_read(interface, REG_INT_STATUS);
    data = INT_ENERGY;
    ksz8851_reg_write(interface, REG_INT_STATUS, data);
    found_INT |= INT_ENERGY;
  }
  /* Resolve the SPI Error interrupt */
  if (ISR_stat & INT_SPI_ERROR)
  {
//    dmc_puts("INT_SPI_ERROR\n");
    /* Clear SPI Error Interrupt flag */
    data = ksz8851_reg_read(interface, REG_INT_STATUS);
    data = INT_SPI_ERROR;
    ksz8851_reg_write(interface, REG_INT_STATUS, data);
    found_INT |= INT_SPI_ERROR;
  }
  /* Resolve the TX space interrupt */
  if (ISR_stat & INT_TX_SPACE)
  {
//    dmc_puts("INT_TX_SPACE\n");
    /* Clear TX space Interrupt flag */
    data = ksz8851_reg_read(interface, REG_INT_STATUS);
    data = INT_TX_SPACE;
    ksz8851_reg_write(interface, REG_INT_STATUS, data);
    found_INT |= INT_TX_SPACE;
  }
  return found_INT;
}

// https://github.com/EnergyMicro/kit_common/blob/master/drivers/ksz8851snl.c
/***************************************************************************//**
 * @brief Returns the size of the currently received frame
 *
 * @return
 *     the printed string
 *
 *****************************************************************************/
uint16_t ksz8851_CurrFrameSize(struct KSZ8851_INTERFACE *interface)
{
  uint16_t data;

  /* Read the byte size of the received frame */
  data = ksz8851_reg_read(interface, REG_RX_FHR_BYTE_CNT);

  data &= RX_BYTE_CNT_MASK;

  return data;
}

/***************************************************************************//**
 * @brief Returns the difference in bytes to be DWORD aligned
 *
 * @param val
 *     value that needs to be aligned
 * @return
 *     the number of bytes needed to be added so that the value is aligned
 *
 *****************************************************************************/
uint8_t ksz8851_DwordAllignDiff(uint8_t val)
{
  if (val % 4 == 0)
  {
    return 0;
  }
  else
  {
    return val % 4;
  }
}

/**
 * @brief CRC calculation
 * @param[in] data Pointer to the data over which to calculate the CRC
 * @param[in] length Number of bytes to process
 * @return Resulting CRC value
 **/
uint32_t ksz8851_CalcCrc(const void *data, size_t length)
{
  uint32_t i;
  uint32_t j;

  //Point to the data over which to calculate the CRC
  const uint8_t *p = (uint8_t *) data;
  //CRC preset value
  uint32_t crc = 0xFFFFFFFF;

  //Loop through data
  for (i = 0; i < length; i++)
  {
    //The message is processed bit by bit
    for (j = 0; j < 8; j++)
    {
      //Update CRC value
      if (((crc >> 31) ^ (p[i] >> j)) & 0x01)
        crc = (crc << 1) ^ 0x04C11DB7;
      else
        crc = crc << 1;
    }
  }

  //Return CRC value
  return crc;
}

void ksz8851_irq(struct KSZ8851_INTERFACE *interface)
{
  uint16_t isr = ksz8851_reg_read(interface, REG_INT_STATUS);

  if (isr == 0)
  {
    return;
  }
//  if (interface->isr_old == isr)
//  {
//    return;
//  }
//  dmc_puts("ksz8851_isr: ");
//  dmc_puthex4(isr);
//  dmc_puts("\n");

  if (isr & IRQ_LCI)
  {
    // LCIS Link Change Interrupt Status
    // When this bit is set, it indicates that the link status has changed from link
    // up to link down, or link down to link up.
    // This edge-triggered interrupt status is cleared by writing 1 to this bit.
    dmc_puts("LCI ");
    if (isr & IRQ_LDI)
    {
      // LDIS Linkup Detect Interrupt Status
      // When this bit is set, it indicates that wake-up from linkup detect status
      // has occurred. Write “0010” to PMECR[5:2] to clear this bit.
      dmc_puts("LDI (Link up)\n");
        uint16_t PHYStatusGet = ksz8851_PHYStatusGet(interface);
//        if (PHYStatusGet & (1 < 15))
//          dmc_puts("HP Auto MDI-X mode.\n");
        if (PHYStatusGet & (1 < 10))
          dmc_puts("100 Mbps\n");
        if (PHYStatusGet & (1 < 9))
          dmc_puts("Full Duplex\n");
        dmc_puthex4cr(PHYStatusGet);
    }
    else
    {
      ksz8851_reg_write(interface, REG_INT_STATUS, isr);
      dmc_puts("(Link down)\n");
    }
//    ksz8851_reg_write(interface, REG_INT_STATUS, isr);
  }
  else
  if (isr & IRQ_LDI)
  {
    // LDIS Linkup Detect Interrupt Status
    // When this bit is set, it indicates that wake-up from linkup detect status
    // has occurred. Write “0010” to PMECR[5:2] to clear this bit.
//    ksz8851_reg_write(interface, REG_INT_STATUS, isr);
    dmc_puts("LDI (Link up)\n");
  }
  if (isr & IRQ_TXI)
  {
    // TXIS Transmit Interrupt Status
    // When this bit is set, it indicates that the TXQ MAC has transmitted at
    // least a frame on the MAC interface and the QMU TXQ is ready for new
    // frames from the host.
    // This edge-triggered interrupt status is cleared by writing 1 to this bit.
//    ksz8851_reg_write(interface, REG_INT_STATUS, isr);
    dmc_puts("TXI\n");
//    led_net_tx();
  }
  if (isr & IRQ_RXI)
  {
    // When this bit is set, it indicates that the QMU RXQ has received at least
    // a frame from the MAC interface and the frame is ready for the host CPU
    // to process.
    dmc_puts("RXI\n");
      dmc_puts("CurrFrameSize: ");
    uint16_t CurrFrameSize = ksz8851_CurrFrameSize(interface);
    dmc_putintcr(CurrFrameSize);

    ksz8851snl_reset_rx(interface);

//    led_net_rx();
  }
  if (isr & IRQ_TXPSI)
  {
    // TXPSIS Transmit Process Stopped Interrupt Status
    // When this bit is set, it indicates that the Transmit Process has stopped.
    // This edge-triggered interrupt status is cleared by writing 1 to this bit.
//    ksz8851_reg_write(interface, REG_INT_STATUS, isr);
    dmc_puts("IRQ_TXPSI\n");
  }
  if (isr & IRQ_RXPSI)
  {
    // RXPSIS Receive Process Stopped Interrupt Status
    // When this bit is set, it indicates that the Receive Process has stopped.
    // This edge-triggered interrupt status is cleared by writing 1 to this bit.
    dmc_puts("IRQ_RXPSI\n");
  }
  if (isr & IRQ_SPIBEI)
  {
    // SPIBEIS SPI Bus Error Interrupt Status
    // When this bit is set, it indicates that SPI bus error status has occurred.
    // This edge-triggered interrupt status is cleared by writing 1 to this bit.
//    ksz8851_reg_write(interface, REG_INT_STATUS, isr);
    dmc_puts("SPIBEI\n");
  }

//  ksz8851_reg_write(interface, REG_INT_STATUS, isr);

  // Write 1 (0xFFFF) to clear all interrupt status bits after an interrupt
  // occurred in Interrupt Status Register.

  ksz8851snl_reset_rx(interface);

  ksz8851_reg_write(interface, REG_INT_STATUS, 0xFFFF);
  ksz8851_reg_write(interface, REG_POWER_CNTL, 0x003C);

  interface->isr_old = isr;
}

void ksz8851_clr_irq(struct KSZ8851_INTERFACE *interface)
{
  uint16_t isr = ksz8851_reg_read(interface, REG_INT_STATUS);

  ksz8851_reg_write(interface, REG_INT_STATUS, isr);
//  ksz8851_reg_write(interface, REG_INT_STATUS, 0xFFFF);
//  ksz8851_reg_write(interface, REG_POWER_CNTL, 0x003C);
}

void ksz8851_ClearRxInterrupt(struct KSZ8851_INTERFACE *interface)
{
  // Clear RXIS Receive Interrupt Status
  ksz8851_reg_setbits(interface, INT_STATUS_REG, INT_RX);
}

void ksz8851_EnableRxInterrupt(struct KSZ8851_INTERFACE *interface)
{
  // Clear RXIS Receive Interrupt Status
  ksz8851_reg_setbits(interface, INT_STATUS_REG, INT_RX);
  // Set RXIE Receive Interrupt Enable
  ksz8851_reg_setbits(interface, INT_ENABLE_REG, INT_RX);
}

void ksz8851_DisableRxInterrupt(struct KSZ8851_INTERFACE *interface)
{
  // Clr RXIE Receive Interrupt Enable
  ksz8851_reg_clrbits(interface, INT_ENABLE_REG, INT_RX);
}

uint16_t ksz8851_ReadRxFrameCount(struct KSZ8851_INTERFACE *interface)
{
  uint16_t rxfctr;
  // Bit 15-8
  // RXFC RX Frame Count
  // To indicate the total received frames in RXQ frame buffer when receive
  // interrupt (bit13=1 in ISR) occurred and write “1” to clear this bit 13 in
  // ISR. The host CPU can start to read the updated receive frame header
  // information in RXFHSR/RXFHBCR registers after read this RX frame
  // count register.
  rxfctr = ksz8851_reg_read(interface, KS_RXFCTR);
  rxfctr = rxfctr >> 8;
  return rxfctr;
}

uint16_t ksz8851_ReadRxByteCount(struct KSZ8851_INTERFACE *interface)
{
  uint16_t rxfhbcr;
  rxfhbcr = ksz8851_reg_read(interface, REG_RX_FHR_BYTE_CNT);
  rxfhbcr &= 0x0fff;
  return rxfhbcr;
}

uint16_t ksz8851_ReadRxHeaderStatus(struct KSZ8851_INTERFACE *interface)
{
  uint16_t rxfhsr;
  rxfhsr = ksz8851_reg_read(interface, REG_RX_FHR_STATUS);
  rxfhsr &= 0xBCFF; // Mask out reserved bits 1011 1100 1111 1111
  // Bit 15 RXFV Receive Frame Valid
  // Bit 14 Reserved
  // Bit 13 RXICMPFCS Receive ICMP Frame Checksum Status
  // Bit 12 RXIPFCS Receive IP Frame Checksum Status
  // Bit 11 RXTCPFCS Receive TCP Frame Checksum Status
  // Bit 10 RXUDPFCS Receive UDP Frame Checksum Status
  // Bit  9 Reserved
  // Bit  8 Reserved
  // Bit  7 RXBF Receive Broadcast Frame
  // Bit  6 RXMF Receive Multicast Frame
  // Bit  5 RXUF Receive Unicast Frame
  // Bit  4 RXMR Receive MII Error
  // Bit  3 RXFT Receive Frame Type
  // Bit  2 RXFTL Receive Frame Too Long
  // Bit  1 RXRF Receive Runt Frame
  // Bit  0 RXCE Receive CRC Error
  return rxfhsr;
}

void ksz8851_ClearRxFramePointer(struct KSZ8851_INTERFACE *interface)
{
  // The value of this register determines the address to be accessed within the RXQ frame buffer. When the Auto Increment
  // is set, it will automatically increment the RXQ Pointer on read accesses to the data register.
  // The counter is incremented is by one for every byte access, by two for every word access, and by four for every double
  // word access.
  uint16_t rxfdpr;
  rxfdpr = ksz8851_reg_read(interface, RX_FD_PTR_REG);
  rxfdpr &= ADDR_PTR_AUTO_INC;  // Save ADDR_PTR_AUTO_INC bit, clear 0x07ff = pointer mask
  ksz8851_reg_write(interface, RX_FD_PTR_REG, rxfdpr);
}

void ksz8851_SetRxFramePointerAutoIncrement(struct KSZ8851_INTERFACE *interface)
{
  uint16_t rxfdpr;
  rxfdpr = ksz8851_reg_read(interface, RX_FD_PTR_REG);
  rxfdpr |= ADDR_PTR_AUTO_INC;
  ksz8851_reg_write(interface, RX_FD_PTR_REG, rxfdpr);
}

void ksz8851_ClrRxFramePointerAutoIncrement(struct KSZ8851_INTERFACE *interface)
{
  uint16_t rxfdpr;
  rxfdpr = ksz8851_reg_read(interface, RX_FD_PTR_REG);
  rxfdpr &= ~ADDR_PTR_AUTO_INC;
  ksz8851_reg_write(interface, RX_FD_PTR_REG, rxfdpr);
}

void ksz8851_EnableRXQReadAccess(struct KSZ8851_INTERFACE *interface)
{
  // Clear RXIS Receive Interrupt Status
  ksz8851_reg_setbits(interface, RXQ_CMD_REG, RXQ_START);
}

void ksz8851_DisableRXQReadAccess(struct KSZ8851_INTERFACE *interface)
{
  // Clear RXIS Receive Interrupt Status
  ksz8851_reg_clrbits(interface, RXQ_CMD_REG, RXQ_START);
}

uint16_t ksz8851_ReadRxInterruptSource(struct KSZ8851_INTERFACE *interface)
{
  uint16_t data;

  /* Read Rx interrupt source */
  data = ksz8851_reg_read(interface, RXQ_CMD_REG);
  // Mask bits 10-12
  // Bit 10
  // RXFCTS RX Frame Count Threshold Status
  // When this bit is set, it indicates that RX interrupt is due to the number of
  // received frames in RXQ buffer exceeds the threshold set in
  // RX Frame Count Threshold Register (0x9C, RXFCT).
  // This bit will be updated when write 1 to bit 13 in ISR register.
  // Bit 11
  // RXDBCTS RX Data Byte Count Threshold Status
  // When this bit is set, it indicates that RX interrupt is due to the number of
  // received bytes in RXQ buffer exceeds the threshold set in
  // RX Data Byte Count Threshold Register (0x8E, RXDBCT).
  // This bit will be updated when write 1 to bit 13 in ISR register.
  // Bit 12
  // RXDTTS RX Duration Timer Threshold Status
  // When this bit is set, it indicates that RX interrupt is due to the time start
  // at first received frame in RXQ buffer exceeds the threshold set in
  // RX Duration Timer Threshold Register (0x8C, RXDTT).
  // This bit will be updated when write 1 to bit 13 in ISR register.
  data &= 0x1c00;
//  if (data & 0x0400)
//  {
//    dmc_puts("RXFCTS RX Frame Count Threshold Status\n");
//  }
//  if (data & 0x0800)
//  {
//    dmc_puts("RXDBCTS RX Data Byte Count Threshold Status\n");
//  }
//  if (data & 0x1000)
//  {
//    dmc_puts("RXDTTS RX Duration Timer Threshold Status\n");
//  }

  return data;
}

uint16_t ksz8851_read_id(struct KSZ8851_INTERFACE *interface)
{
  uint16_t dev_id = ksz8851_reg_read(interface, REG_CHIP_ID);
  dmc_puts("dev_id: ");
  dmc_puthex4(dev_id);
  return dev_id;
}

uint16_t ksz8851_ReadIntRegisterValue(struct KSZ8851_INTERFACE *interface)
{
  return ksz8851_reg_read(interface, REG_INT_STATUS);
}

void ksz8851_IntHandler(struct KSZ8851_INTERFACE *interface)
{
//  dmc_puts(TERMINAL_LIGHT_BLUE);
//  dmc_puts("kszint\n");
//  dmc_puts(TERMINAL_DEFAULT);


//  interface->isr_reg = ksz8851_CheckIrqStat(interface);
//  return;
//  ksz8851_irq(interface);

//  interface->isr_reg = ksz8851_reg_read(interface, REG_INT_STATUS);

//  if (interface->isr_reg == 0)
//  {
//    return;
//  }
  interface->isr_ocurred = 1;

////  if (interface->isr_old == isr)
////  {
////    return;
////  }
//  dmc_puts("ksz8851_isr: ");
//  dmc_puthex4(interface->isr_reg);
//  dmc_puts("\n");

//  if (isr & IRQ_LCI)
//  {
//    // LCIS Link Change Interrupt Status
//    // When this bit is set, it indicates that the link status has changed from link
//    // up to link down, or link down to link up.
//    // This edge-triggered interrupt status is cleared by writing 1 to this bit.
//    dmc_puts("LCI ");
//    if (isr & IRQ_LDI)
//    {
//      // LDIS Linkup Detect Interrupt Status
//      // When this bit is set, it indicates that wake-up from linkup detect status
//      // has occurred. Write “0010” to PMECR[5:2] to clear this bit.
//      dmc_puts("LDI (Link up)\n");
//        uint16_t PHYStatusGet = ksz8851_PHYStatusGet(interface);
////        if (PHYStatusGet & (1 < 15))
////          dmc_puts("HP Auto MDI-X mode.\n");
//        if (PHYStatusGet & (1 < 10))
//          dmc_puts("100 Mbps\n");
//        if (PHYStatusGet & (1 < 9))
//          dmc_puts("Full Duplex\n");
//        dmc_puthex4cr(PHYStatusGet);
//    }
//    else
//    {
//      ksz8851_reg_write(interface, REG_INT_STATUS, isr);
//      dmc_puts("(Link down)\n");
//    }
////    ksz8851_reg_write(interface, REG_INT_STATUS, isr);
//  }
//  else
//  if (isr & IRQ_LDI)
//  {
//    // LDIS Linkup Detect Interrupt Status
//    // When this bit is set, it indicates that wake-up from linkup detect status
//    // has occurred. Write “0010” to PMECR[5:2] to clear this bit.
////    ksz8851_reg_write(interface, REG_INT_STATUS, isr);
//    dmc_puts("LDI (Link up)\n");
//  }
//  if (isr & IRQ_TXI)
//  {
//    // TXIS Transmit Interrupt Status
//    // When this bit is set, it indicates that the TXQ MAC has transmitted at
//    // least a frame on the MAC interface and the QMU TXQ is ready for new
//    // frames from the host.
//    // This edge-triggered interrupt status is cleared by writing 1 to this bit.
////    ksz8851_reg_write(interface, REG_INT_STATUS, isr);
//    dmc_puts("TXI\n");
////    led_net_tx();
//  }
//  if (isr & IRQ_RXI)
//  {
//    // When this bit is set, it indicates that the QMU RXQ has received at least
//    // a frame from the MAC interface and the frame is ready for the host CPU
//    // to process.
//    dmc_puts("RXI\n");
//      dmc_puts("CurrFrameSize: ");
//    uint16_t CurrFrameSize = ksz8851_CurrFrameSize(interface);
//    dmc_putintcr(CurrFrameSize);
//
//    ksz8851snl_reset_rx(interface);
//
////    led_net_rx();
//  }
//  if (isr & IRQ_TXPSI)
//  {
//    // TXPSIS Transmit Process Stopped Interrupt Status
//    // When this bit is set, it indicates that the Transmit Process has stopped.
//    // This edge-triggered interrupt status is cleared by writing 1 to this bit.
////    ksz8851_reg_write(interface, REG_INT_STATUS, isr);
//    dmc_puts("IRQ_TXPSI\n");
//  }
//  if (isr & IRQ_RXPSI)
//  {
//    // RXPSIS Receive Process Stopped Interrupt Status
//    // When this bit is set, it indicates that the Receive Process has stopped.
//    // This edge-triggered interrupt status is cleared by writing 1 to this bit.
//    dmc_puts("IRQ_RXPSI\n");
//  }
//  if (isr & IRQ_SPIBEI)
//  {
//    // SPIBEIS SPI Bus Error Interrupt Status
//    // When this bit is set, it indicates that SPI bus error status has occurred.
//    // This edge-triggered interrupt status is cleared by writing 1 to this bit.
////    ksz8851_reg_write(interface, REG_INT_STATUS, isr);
//    dmc_puts("SPIBEI\n");
//  }
//
////  ksz8851_reg_write(interface, REG_INT_STATUS, isr);
//
//  // Write 1 (0xFFFF) to clear all interrupt status bits after an interrupt
//  // occurred in Interrupt Status Register.

//  ksz8851snl_reset_rx(interface);

//  uint16_t usValue = ksz8851_reg_read(interface, REG_RX_CTRL1);
//  usValue &= ~( ( uint16_t ) RX_CTRL_ENABLE | RX_CTRL_FLUSH_QUEUE );
//  ksz8851_reg_write(interface, REG_RX_CTRL1, usValue );
////  HAL_Delay(2);
//  ksz8851_reg_write(interface, REG_RX_CTRL1, usValue | RX_CTRL_FLUSH_QUEUE );
////  HAL_Delay(1);
//  ksz8851_reg_write(interface, REG_RX_CTRL1, usValue );
////  HAL_Delay(1);
//  ksz8851_reg_write(interface, REG_RX_CTRL1, usValue | RX_CTRL_ENABLE );
////  HAL_Delay(1);

//  ksz8851_reg_write(interface, REG_INT_STATUS, 0xFFFF);
//  ksz8851_reg_write(interface, REG_POWER_CNTL, 0x003C);
//
//
//  /* Enable INTN flag. */
////  g_intn_flag = 1;
}

uint8_t ksz8851_has_isr_RXI(uint16_t isr_reg)
{
  if (isr_reg & IRQ_RXI)
  {
    return TRUE;
  }
  return FALSE;
}

void ksz8851_show_isr(struct KSZ8851_INTERFACE *interface, uint16_t isr_reg)
{
  if (isr_reg & IRQ_LCI)
  {
    // LCIS Link Change Interrupt Status
    // When this bit is set, it indicates that the link status has changed from link
    // up to link down, or link down to link up.
    // This edge-triggered interrupt status is cleared by writing 1 to this bit.
    dmc_puts("LCI ");
    if (isr_reg & IRQ_LDI)
    {
      // LDIS Linkup Detect Interrupt Status
      // When this bit is set, it indicates that wake-up from linkup detect status
      // has occurred. Write “0010” to PMECR[5:2] to clear this bit.
      dmc_puts("LDI (Link up)\n");
      uint16_t PHYStatusGet = ksz8851_PHYStatusGet(interface);
      //        if (PHYStatusGet & (1 < 15))
      //          dmc_puts("HP Auto MDI-X mode.\n");
      if (PHYStatusGet & (1 < 10))
        dmc_puts("100 Mbps\n");
      if (PHYStatusGet & (1 < 9))
        dmc_puts("Full Duplex\n");
      dmc_puthex4cr(PHYStatusGet);
    }
    else
    {
      ksz8851_reg_write(interface, REG_INT_STATUS, isr_reg);
      dmc_puts("(Link down)\n");
    }
    //    ksz8851_reg_write(interface, REG_INT_STATUS, isr);
  }
  else
  if (isr_reg & IRQ_LDI)
  {
    // LDIS Linkup Detect Interrupt Status
    // When this bit is set, it indicates that wake-up from linkup detect status
    // has occurred. Write “0010” to PMECR[5:2] to clear this bit.
    //    ksz8851_reg_write(interface, REG_INT_STATUS, isr);
    dmc_puts("LDI (Link up)\n");
  }
  if (isr_reg & IRQ_TXI)
  {
    // TXIS Transmit Interrupt Status
    // When this bit is set, it indicates that the TXQ MAC has transmitted at
    // least a frame on the MAC interface and the QMU TXQ is ready for new
    // frames from the host.
    // This edge-triggered interrupt status is cleared by writing 1 to this bit.
    //    ksz8851_reg_write(interface, REG_INT_STATUS, isr);
    dmc_puts("TXI\n");
    //    led_net_tx();
  }
  if (isr_reg & IRQ_RXI)
  {
    // When this bit is set, it indicates that the QMU RXQ has received at least
    // a frame from the MAC interface and the frame is ready for the host CPU
    // to process.
    dmc_puts("RXI\n");
    dmc_puts("CurrFrameSize: ");
    uint16_t CurrFrameSize = ksz8851_CurrFrameSize(interface);
    dmc_putintcr(CurrFrameSize);

    ksz8851snl_reset_rx(interface);

    //    led_net_rx();
  }
  if (isr_reg & IRQ_TXPSI)
  {
    // TXPSIS Transmit Process Stopped Interrupt Status
    // When this bit is set, it indicates that the Transmit Process has stopped.
    // This edge-triggered interrupt status is cleared by writing 1 to this bit.
    //    ksz8851_reg_write(interface, REG_INT_STATUS, isr);
    dmc_puts("IRQ_TXPSI\n");
  }
  if (isr_reg & IRQ_RXPSI)
  {
    // RXPSIS Receive Process Stopped Interrupt Status
    // When this bit is set, it indicates that the Receive Process has stopped.
    // This edge-triggered interrupt status is cleared by writing 1 to this bit.
    dmc_puts("IRQ_RXPSI\n");
  }
  if (isr_reg & IRQ_SPIBEI)
  {
    // SPIBEIS SPI Bus Error Interrupt Status
    // When this bit is set, it indicates that SPI bus error status has occurred.
    // This edge-triggered interrupt status is cleared by writing 1 to this bit.
    //    ksz8851_reg_write(interface, REG_INT_STATUS, isr);
    dmc_puts("SPIBEI\n");
  }

}

bool ksz8851_CheckQMUTXQHasSpace(struct KSZ8851_INTERFACE *interface, uint16_t length)
{
  uint16_t txmir;
  uint16_t reqSize;

  // The Ethernet packet data length does not include CRC.
  // Read value from TXMIR to check if QMU TXQ has enough amount of memory for
  // the Ethernet packet data plus 4-byte frame header, plus 4-byte for DWORD alignment.
  // Compare the read value with (txPacketLength+4+4)
  /* Wait for previous frame to finish before setting up a new one */
  while (ksz8851_reg_read(interface, REG_TXQ_CMD) & TXQ_ENQUEUE)
  {
    dmc_puts("wait...\n");
  }

  /* Make sure there is room for
   *
   * 4 bytes Control Word
   * 4 bytes Byte Count
   * n bytes Packet
   * 4 bytes CRC (<- NO) 12 -> 8
   */
  reqSize = length + 8;
  txmir = ksz8851_reg_read(interface, REG_TX_MEM_INFO) & TX_MEM_AVAIL_MASK;
//  LWIP_DEBUGF(NETIF_DEBUG, ("KSZ8851SNL_LongTransmitInit: txmir =%hu  reqSize = %hu \n", txmir, reqSize));

  if (txmir < reqSize)
  {
    /* TXQ is out of memory */
//    LWIP_DEBUGF(NETIF_DEBUG | LWIP_DBG_LEVEL_WARNING,
//        ("Not enough TXQ Memory, available=%u required=%u\n", txmir, reqSize));
    return false;
  }
  return true;
}

void clr_dma_tx_ended(struct KSZ8851_INTERFACE *interface)
{
  dmc_puts("clr_dma_tx_ended\n");
  interface->dma_tx_ended = 0;
}

void set_dma_tx_ended(struct KSZ8851_INTERFACE *interface)
{
  dmc_puts("set_dma_tx_ended\n");
  interface->dma_tx_ended = 1;
}

void wait_dma_tx_ended(struct KSZ8851_INTERFACE *interface)
{
  dmc_puts("wait_dma_tx_ended\n");
  while (interface->dma_tx_ended != 1);
}

void clr_dma_rx_ended(struct KSZ8851_INTERFACE *interface)
{
  dmc_puts("clr_dma_rx_ended\n");
  interface->dma_rx_ended = 0;
}

void set_dma_rx_ended(struct KSZ8851_INTERFACE *interface)
{
  dmc_puts("set_dma_rx_ended\n");
  interface->dma_rx_ended = 1;
}

void wait_dma_rx_ended(struct KSZ8851_INTERFACE *interface)
{
  dmc_puts("wait_dma_rx_ended\n");
  while (interface->dma_rx_ended != 1);
}

uint32_t KSZ8851_GetLinkState(struct KSZ8851_INTERFACE *interface)
{
  uint16_t port_stat = ksz8851_reg_read(interface, REG_PORT_STATUS); /* P1SR */

  //Check link state
  if(port_stat & PORT_STATUS_LINK_GOOD)
  {
    //Link is up
    interface->link = KSZ8851_LINK_STATE_UP;

    //Get current speed
    if(port_stat & PORT_STAT_SPEED_100MBIT)
    {
      interface->speed = KSZ8851_LINK_SPEED_100MB;
      //interface->linkSpeed = NIC_LINK_SPEED_100MBPS;
    }
    else
    {
      interface->speed = KSZ8851_LINK_SPEED_10MB;
      //interface->linkSpeed = NIC_LINK_SPEED_10MBPS;
    }

    //Determine the new duplex mode
    if(port_stat & PORT_STAT_FULL_DUPLEX)
    {
      interface->duplex = KSZ8851_LINK_DUPLEX_FULL;
      // interface->duplexMode = NIC_FULL_DUPLEX_MODE;
    }
    else
    {
      interface->duplex = KSZ8851_LINK_DUPLEX_HALF;
      // interface->duplexMode = NIC_HALF_DUPLEX_MODE;
    }

    if ((interface->speed == KSZ8851_LINK_SPEED_100MB) && (interface->duplex == KSZ8851_LINK_DUPLEX_FULL))
    {
      interface->state = KSZ8851_STATUS_100MBITS_FULLDUPLEX;
    }
    if ((interface->speed == KSZ8851_LINK_SPEED_10MB) && (interface->duplex == KSZ8851_LINK_DUPLEX_FULL))
    {
      interface->state = KSZ8851_STATUS_10MBITS_FULLDUPLEX;
    }
    if ((interface->speed == KSZ8851_LINK_SPEED_100MB) && (interface->duplex == KSZ8851_LINK_DUPLEX_HALF))
    {
      interface->state = KSZ8851_STATUS_100MBITS_HALFDUPLEX;
    }
    if ((interface->speed == KSZ8851_LINK_SPEED_10MB) && (interface->duplex == KSZ8851_LINK_DUPLEX_HALF))
    {
      interface->state = KSZ8851_STATUS_10MBITS_HALFDUPLEX;
    }
  }
  else
  {
    //Link is down
    interface->state = KSZ8851_STATUS_LINK_DOWN;
  }

  return interface->state;
}

