#ifndef __SNTP_CLIENT_H
#define __SNTP_CLIENT_H

/* C++ detection */
#ifdef __cplusplus
extern "C"
{
#endif

#include "stm32h7xx_hal.h"

void SNTP_ObtainTime(void);
int SNTP_SetServerName(const char* serverName);
int SNTP_Init(void);
void SNTP_Deinit(void);
void setTimeZone(long offset, int daylight);

/* C++ detection */
#ifdef __cplusplus
}
#endif

#endif /* __SNTP_CLIENT_H */
