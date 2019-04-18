#include "ksz8851snl_1.h"

/**
 * @brief KSZ8851 initialization
 * @param None
 * @return None
 */
uint8_t ksz8851_init_1(void)
{
  uint8_t MAC_address[] = { 0x00, 0x80, 0xe1, 0x00, 0x00, 0x02 };

  ksz8851_init(&KSZ8851_interface_1, &hspi4, SPI4_CS_GPIO_Port, SPI4_CS_Pin, 0, 0, MAC_address);

  return 1;
}

void ksz8851_IntEnable_1(void)
{
  ksz8851_IntEnable(&KSZ8851_interface_1);
}

void ksz8851snl_reset_tx_1(void)
{
  ksz8851snl_reset_tx(&KSZ8851_interface_1);
}

void ksz8851snl_reset_rx_1(void)
{
  ksz8851snl_reset_rx(&KSZ8851_interface_1);
}

void ksz8851_IntClearAll_1(void)
{
  ksz8851_IntClearAll(&KSZ8851_interface_1);
}

void ksz8851_Send_1(uint8_t *pTXData, uint16_t pTXLength)
{
  ksz8851_Send(&KSZ8851_interface_1, pTXData, pTXLength);
}

uint16_t ksz8851_Receive_1(uint8_t *pRXData, uint16_t pRXLength)
{
  return ksz8851_Receive(&KSZ8851_interface_1, pRXData, pRXLength);
}

uint32_t KSZ8851_1_GetLinkState()
{
  return KSZ8851_GetLinkState(&KSZ8851_interface_1);
}
