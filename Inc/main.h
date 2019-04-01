/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "dmc_defines.h"
#include "dmc_terminal.h"

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);
void _Error_Handler(char *file, int line);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define SPI1_CS_Pin GPIO_PIN_4
#define SPI1_CS_GPIO_Port GPIOA
#define SPI4_CS_Pin GPIO_PIN_11
#define SPI4_CS_GPIO_Port GPIOE

#define RMII_MDC_Pin GPIO_PIN_1       // 16
#define RMII_MDC_GPIO_Port GPIOC
#define RMII_REF_CLK_Pin GPIO_PIN_1   // 23
#define RMII_REF_CLK_GPIO_Port GPIOA
#define RMII_MDIO_Pin GPIO_PIN_2      // 24
#define RMII_MDIO_GPIO_Port GPIOA

// MODE0
#define RMII_RXD0_Pin GPIO_PIN_4      // 32
#define RMII_RXD0_GPIO_Port GPIOC
// MODE1
#define RMII_RXD1_Pin GPIO_PIN_5      // 33
#define RMII_RXD1_GPIO_Port GPIOC
// MODE2
#define RMII_CRS_DV_Pin GPIO_PIN_7    // 31
#define RMII_CRS_DV_GPIO_Port GPIOA

#define RMII_TX_EN_Pin GPIO_PIN_11    // 47
#define RMII_TX_EN_GPIO_Port GPIOB
#define RMII_TXD0_Pin GPIO_PIN_12     // 51
#define RMII_TXD0_GPIO_Port GPIOB
#define RMII_TXD1_Pin GPIO_PIN_13     // 52
#define RMII_TXD1_GPIO_Port GPIOB

#define ETH_RCLK_Pin GPIO_PIN_1       // 23
#define ETH_RCLK_GPIO_Port GPIOA
#define ETH_CRS_Pin GPIO_PIN_7        // 31
#define ETH_CRS_GPIO_Port GPIOA
#define ETH_TXEN_Pin GPIO_PIN_11      // 47
#define ETH_TXEN_GPIO_Port GPIOB

#define QSPI_IO2_Pin GPIO_PIN_2
#define QSPI_IO2_GPIO_Port GPIOE
#define S1_1_Pin GPIO_PIN_3
#define S1_1_GPIO_Port GPIOE
#define S1_2_Pin GPIO_PIN_4
#define S1_2_GPIO_Port GPIOE
#define S1_3_Pin GPIO_PIN_5
#define S1_3_GPIO_Port GPIOE
#define S1_4_Pin GPIO_PIN_6
#define S1_4_GPIO_Port GPIOE
#define LED_RUN_Pin GPIO_PIN_0
#define LED_RUN_GPIO_Port GPIOC
#define MDC_Pin GPIO_PIN_1
#define MDC_GPIO_Port GPIOC
#define INTRN1_Pin GPIO_PIN_0
#define INTRN1_GPIO_Port GPIOA
#define INTRN1_EXTI_IRQn EXTI0_IRQn
#define PME1_Pin GPIO_PIN_3
#define PME1_GPIO_Port GPIOA
#define PME1_EXTI_IRQn EXTI3_IRQn
#define QSPI_CLK_Pin GPIO_PIN_2
#define QSPI_CLK_GPIO_Port GPIOB
#define UART7_TXE_Pin GPIO_PIN_9
#define UART7_TXE_GPIO_Port GPIOE
#define QSPI_NCS_Pin GPIO_PIN_10
#define QSPI_NCS_GPIO_Port GPIOB
#define HS_N_Pin GPIO_PIN_14
#define HS_N_GPIO_Port GPIOB
#define HS_P_Pin GPIO_PIN_15
#define HS_P_GPIO_Port GPIOB
#define PME2_Pin GPIO_PIN_8
#define PME2_GPIO_Port GPIOD
#define PME2_EXTI_IRQn EXTI9_5_IRQn
#define INTRN2_Pin GPIO_PIN_9
#define INTRN2_GPIO_Port GPIOD
#define INTRN2_EXTI_IRQn EXTI9_5_IRQn
#define SW1_Pin GPIO_PIN_10
#define SW1_GPIO_Port GPIOD
#define QSPI_IO0_Pin GPIO_PIN_11
#define QSPI_IO0_GPIO_Port GPIOD
#define QSPI_IO1_Pin GPIO_PIN_12
#define QSPI_IO1_GPIO_Port GPIOD
#define QSPI_IO3_Pin GPIO_PIN_13
#define QSPI_IO3_GPIO_Port GPIOD
#define SW2_Pin GPIO_PIN_14
#define SW2_GPIO_Port GPIOD
#define SW3_Pin GPIO_PIN_15
#define SW3_GPIO_Port GPIOD
#define LED_CAN1_OK_Pin GPIO_PIN_6
#define LED_CAN1_OK_GPIO_Port GPIOC
#define LED_CAN2_OK_Pin GPIO_PIN_7
#define LED_CAN2_OK_GPIO_Port GPIOC
#define SD_D0_Pin GPIO_PIN_8
#define SD_D0_GPIO_Port GPIOC
#define SD_D1_Pin GPIO_PIN_9
#define SD_D1_GPIO_Port GPIOC
#define LED_RS1_OK_Pin GPIO_PIN_8
#define LED_RS1_OK_GPIO_Port GPIOA
#define LED_RS1_ERR_Pin GPIO_PIN_9
#define LED_RS1_ERR_GPIO_Port GPIOA
#define LED_RS2_OK_Pin GPIO_PIN_10
#define LED_RS2_OK_GPIO_Port GPIOA
#define LED_RS2_ERR_Pin GPIO_PIN_11
#define LED_RS2_ERR_GPIO_Port GPIOA
#define VBUS_IN_Pin GPIO_PIN_12
#define VBUS_IN_GPIO_Port GPIOA
#define SD_D2_Pin GPIO_PIN_10
#define SD_D2_GPIO_Port GPIOC
#define SD_D3_Pin GPIO_PIN_11
#define SD_D3_GPIO_Port GPIOC
#define SD_CK_Pin GPIO_PIN_12
#define SD_CK_GPIO_Port GPIOC
#define CAN1_RX_Pin GPIO_PIN_0
#define CAN1_RX_GPIO_Port GPIOD
#define CAN1_TX_Pin GPIO_PIN_1
#define CAN1_TX_GPIO_Port GPIOD
#define SD_CMD_Pin GPIO_PIN_2
#define SD_CMD_GPIO_Port GPIOD
#define SW4_Pin GPIO_PIN_3
#define SW4_GPIO_Port GPIOD
#define USART2_TXE_Pin GPIO_PIN_4
#define USART2_TXE_GPIO_Port GPIOD
#define CAN2_RX_Pin GPIO_PIN_5
#define CAN2_RX_GPIO_Port GPIOB
#define CAN2_TX_Pin GPIO_PIN_6
#define CAN2_TX_GPIO_Port GPIOB
#define CAN2_EN_Pin GPIO_PIN_0
#define CAN2_EN_GPIO_Port GPIOE
#define CAN1_EN_Pin GPIO_PIN_1
#define CAN1_EN_GPIO_Port GPIOE
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
