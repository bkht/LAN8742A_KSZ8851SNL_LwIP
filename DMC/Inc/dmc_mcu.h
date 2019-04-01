#ifndef __DMC_MCU_H
#define __DMC_MCU_H

/* C++ detection */
#ifdef __cplusplus
extern "C"
{
#endif

#include "stm32h7xx_hal.h"

char* GetMCUFamily(void);
char* GetMCUType(void);

/* C++ detection */
#ifdef __cplusplus
}
#endif

#endif /* __DMC_MCU_H */
