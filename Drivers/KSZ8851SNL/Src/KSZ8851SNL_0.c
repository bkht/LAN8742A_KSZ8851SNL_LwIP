#include "ksz8851snl_0.h"

/**
 * @brief KSZ8851 initialization
 * @param None
 * @return None
 */
uint8_t ksz8851_init_0(void)
{
  uint8_t MAC_address[] = { 0x00, 0x80, 0xe1, 0x00, 0x00, 0x01 };

  ksz8851_init(&KSZ8851_interface_0, &hspi1, SPI1_CS_GPIO_Port, SPI1_CS_Pin, 0, 0, MAC_address);

  return 1;
}

void ksz8851_IntEnable_0(void)
{
  ksz8851_IntEnable(&KSZ8851_interface_0);
}

void ksz8851snl_reset_tx_0(void)
{
  ksz8851snl_reset_tx(&KSZ8851_interface_0);
}

void ksz8851snl_reset_rx_0(void)
{
  ksz8851snl_reset_rx(&KSZ8851_interface_0);
}

void ksz8851_IntClearAll_0(void)
{
  ksz8851_IntClearAll(&KSZ8851_interface_0);
}

void ksz8851_Send_0(uint8_t *pTXData, uint16_t pTXLength)
{
  ksz8851_Send(&KSZ8851_interface_0, pTXData, pTXLength);
}

uint16_t ksz8851_Receive_0(uint8_t *pRXData, uint16_t pRXLength)
{
  return ksz8851_Receive(&KSZ8851_interface_0, pRXData, pRXLength);
}

uint32_t KSZ8851_0_GetLinkState()
{
  return KSZ8851_GetLinkState(&KSZ8851_interface_0);
}
