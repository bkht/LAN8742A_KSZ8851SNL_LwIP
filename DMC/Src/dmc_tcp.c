#include "dmc_tcp.h"

uint16_t mb_set_exception(uint8_t *buf, uint8_t exc)
{
  buf[MB_HDR_OFFSET + MB_HDR_FC] |= 0x80;
  buf[MB_HDR_OFFSET + MB_HDR_EXCEPTION_CODE] = exc;
  uint16_t len = 3;

  buf[MB_HDR_OFFSET + MB_HDR_LENGTH] = len << 8;
  buf[MB_HDR_OFFSET + MB_HDR_LENGTH + 1] = len & 0xff;
  len += 46;
  buf[IP_HDR_OFFSET + IP_HDR_TOTAL_LENGTH] = len << 8;
  buf[IP_HDR_OFFSET + IP_HDR_TOTAL_LENGTH + 1] = len & 0xff;
  // return total length
  return len + IP_HDR_OFFSET;
}

uint16_t mb_set_len(uint8_t *buf, uint16_t len)
{
  buf[MB_HDR_OFFSET + MB_HDR_LENGTH] = len << 8;
  buf[MB_HDR_OFFSET + MB_HDR_LENGTH + 1] = len & 0xff;
  len += 46;
  buf[IP_HDR_OFFSET + IP_HDR_TOTAL_LENGTH] = len << 8;
  buf[IP_HDR_OFFSET + IP_HDR_TOTAL_LENGTH + 1] = len & 0xff;
  // return total length
  return len + IP_HDR_OFFSET;
}

uint16_t mb_set_request(uint8_t *buf, uint16_t reg, uint16_t reg_count)
{
  uint16_t len = 6;
  memcpy(&buf[MB_HDR_OFFSET + MB_HDR_LENGTH], &len, 2);
  memcpy(&buf[MB_HDR_OFFSET + MB_HDR_REG], &reg, 2);
  memcpy(&buf[MB_HDR_OFFSET + MB_HDR_REG_COUNT], &reg_count, 2);

  buf[MB_HDR_OFFSET + MB_HDR_LENGTH] = len << 8;
  buf[MB_HDR_OFFSET + MB_HDR_LENGTH + 1] = len & 0xff;
  len += 46;
  buf[IP_HDR_OFFSET + IP_HDR_TOTAL_LENGTH] = len << 8;
  buf[IP_HDR_OFFSET + IP_HDR_TOTAL_LENGTH + 1] = len & 0xff;
  // return total length
  return len + IP_HDR_OFFSET;
}

uint16_t mb_set_reply(uint8_t *buf, uint8_t *bytes, uint8_t byte_count)
{
//  uint16_t len = byte_count + 3;
//  buf[MB_HDR_OFFSET + MB_HDR_BYTE_COUNT] = byte_count;

//  for (uint16_t i = 0; i < byte_count; i++)
//  {
//    buf[MB_HDR_OFFSET + MB_HDR_UI + i] = bytes[i];
//  }
  // Alter IP ID
  buf[IP_HDR_OFFSET + 4] = 0x8f;
  buf[IP_HDR_OFFSET + 5] = 0x52;

  buf[MB_HDR_OFFSET + MB_HDR_UI + 0] = 0x01;
  buf[MB_HDR_OFFSET + MB_HDR_UI + 1] = 0x10;
  buf[MB_HDR_OFFSET + MB_HDR_UI + 2] = 0x00;
  buf[MB_HDR_OFFSET + MB_HDR_UI + 3] = 0x00;
  buf[MB_HDR_OFFSET + MB_HDR_UI + 4] = 0x00;
  buf[MB_HDR_OFFSET + MB_HDR_UI + 5] = 0x01;
//  memcpy(&buf[MB_HDR_OFFSET + MB_HDR_BYTES], bytes, byte_count);

  buf[MB_HDR_OFFSET + MB_HDR_LENGTH] = byte_count << 8;
  buf[MB_HDR_OFFSET + MB_HDR_LENGTH + 1] = byte_count & 0xff;
  byte_count += 46;
  buf[IP_HDR_OFFSET + IP_HDR_TOTAL_LENGTH] = byte_count << 8;
  buf[IP_HDR_OFFSET + IP_HDR_TOTAL_LENGTH + 1] = byte_count & 0xff;
  // return total length
  return byte_count + IP_HDR_OFFSET;
}

void swap_buf_all(uint8_t *buf)
{
  uint8_t tmp[6];

  // Swap MAC
  memcpy(tmp, &buf[ETH_HDR_DEST_MAC], ETH_HDR_SIZE_MAC);
  memcpy(&buf[ETH_HDR_DEST_MAC], &buf[ETH_HDR_SOURCE_MAC], ETH_HDR_SIZE_MAC);
  memcpy(&buf[ETH_HDR_SOURCE_MAC], tmp, ETH_HDR_SIZE_MAC);

  // Swap IP
  memcpy(tmp, &buf[IP_HDR_OFFSET + IP_HDR_DEST_IP], IP_HDR_SIZE_IP);
  memcpy(&buf[IP_HDR_OFFSET + IP_HDR_DEST_IP], &buf[IP_HDR_OFFSET + IP_HDR_SOURCE_IP], IP_HDR_SIZE_IP);
  memcpy(&buf[IP_HDR_OFFSET + IP_HDR_SOURCE_IP], tmp, IP_HDR_SIZE_IP);

  // Swap Port
  memcpy(tmp, &buf[TCP_HDR_OFFSET + TCP_HDR_DEST_PORT], TCP_HDR_SIZE_PORT);
  memcpy(&buf[TCP_HDR_OFFSET + TCP_HDR_DEST_PORT], &buf[TCP_HDR_OFFSET + TCP_HDR_SOURCE_PORT], TCP_HDR_SIZE_PORT);
  memcpy(&buf[TCP_HDR_OFFSET + TCP_HDR_SOURCE_PORT], tmp, TCP_HDR_SIZE_PORT);

  // Swap SEQ ACK Number
  memcpy(tmp, &buf[TCP_HDR_OFFSET + TCP_HDR_SEQ_NUMBER], TCP_HDR_SIZE_SEQ);
  memcpy(&buf[TCP_HDR_OFFSET + TCP_HDR_SEQ_NUMBER], &buf[TCP_HDR_OFFSET + TCP_HDR_ACK_NUMBER], TCP_HDR_SIZE_SEQ);
  memcpy(&buf[TCP_HDR_OFFSET + TCP_HDR_ACK_NUMBER], tmp, TCP_HDR_SIZE_SEQ);
  // Alter TCP_HDR_ACK_NUMBER
  buf[TCP_HDR_OFFSET + TCP_HDR_ACK_NUMBER + 3] += 15;
  // Alter Window Size
  buf[TCP_HDR_OFFSET + TCP_HDR_WINDOW_SIZE + 1] += 15;

}

void swap_buf_eth(uint8_t *buf)
{
  uint8_t tmp[6];

  // Swap MAC
  memcpy(tmp, &buf[ETH_HDR_DEST_MAC], ETH_HDR_SIZE_MAC);
  memcpy(&buf[ETH_HDR_DEST_MAC], &buf[ETH_HDR_SOURCE_MAC], ETH_HDR_SIZE_MAC);
  memcpy(&buf[ETH_HDR_SOURCE_MAC], tmp, ETH_HDR_SIZE_MAC);
}

void swap_buf_ip(uint8_t *buf)
{
  uint8_t tmp[4];

  // Swap IP
  memcpy(tmp, &buf[IP_HDR_OFFSET + IP_HDR_DEST_IP], IP_HDR_SIZE_IP);
  memcpy(&buf[IP_HDR_OFFSET + IP_HDR_DEST_IP], &buf[IP_HDR_OFFSET + IP_HDR_SOURCE_IP], IP_HDR_SIZE_IP);
  memcpy(&buf[IP_HDR_OFFSET + IP_HDR_SOURCE_IP], tmp, IP_HDR_SIZE_IP);
}

void swap_buf_tcp(uint8_t *buf)
{
  uint8_t tmp[2];

  // Swap Port
  memcpy(tmp, &buf[TCP_HDR_OFFSET + TCP_HDR_DEST_PORT], TCP_HDR_SIZE_PORT);
  memcpy(&buf[TCP_HDR_OFFSET + TCP_HDR_DEST_PORT], &buf[TCP_HDR_OFFSET + TCP_HDR_SOURCE_PORT], TCP_HDR_SIZE_PORT);
  memcpy(&buf[TCP_HDR_OFFSET + TCP_HDR_SOURCE_PORT], tmp, TCP_HDR_SIZE_PORT);
}
