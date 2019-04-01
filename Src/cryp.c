/**
  ******************************************************************************
  * File Name          : CRYP.c
  * Description        : This file provides code for the configuration
  *                      of the CRYP instances.
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

/* Includes ------------------------------------------------------------------*/
#include "cryp.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

CRYP_HandleTypeDef hcryp;
__ALIGN_BEGIN static const uint32_t pKeyCRYP[6] __ALIGN_END = {
                            0x00000000,0x00000000,0x00000000,0x00000000,0x00000000,0x00000000};
DMA_HandleTypeDef hdma_cryp_in;
DMA_HandleTypeDef hdma_cryp_out;

/* CRYP init function */
void MX_CRYP_Init(void)
{

  hcryp.Instance = CRYP;
  hcryp.Init.DataType = CRYP_DATATYPE_32B;
  hcryp.Init.pKey = (uint32_t *)pKeyCRYP;
  hcryp.Init.Algorithm = CRYP_TDES_ECB;
  if (HAL_CRYP_Init(&hcryp) != HAL_OK)
  {
    Error_Handler();
  }

}

void HAL_CRYP_MspInit(CRYP_HandleTypeDef* crypHandle)
{

  HAL_DMA_MuxSyncConfigTypeDef pSyncConfig= {0};
  if(crypHandle->Instance==CRYP)
  {
  /* USER CODE BEGIN CRYP_MspInit 0 */

  /* USER CODE END CRYP_MspInit 0 */
    /* CRYP clock enable */
    __HAL_RCC_CRYP_CLK_ENABLE();
  
    /* CRYP DMA Init */
    /* CRYP_IN Init */
    hdma_cryp_in.Instance = DMA2_Stream1;
    hdma_cryp_in.Init.Request = DMA_REQUEST_CRYP_IN;
    hdma_cryp_in.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_cryp_in.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_cryp_in.Init.MemInc = DMA_MINC_ENABLE;
    hdma_cryp_in.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_cryp_in.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_cryp_in.Init.Mode = DMA_NORMAL;
    hdma_cryp_in.Init.Priority = DMA_PRIORITY_LOW;
    hdma_cryp_in.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_cryp_in) != HAL_OK)
    {
      Error_Handler();
    }

    pSyncConfig.SyncSignalID = HAL_DMAMUX1_SYNC_EXTI0;
    pSyncConfig.SyncPolarity = HAL_DMAMUX_SYNC_NO_EVENT;
    pSyncConfig.SyncEnable = DISABLE;
    pSyncConfig.EventEnable = DISABLE;
    pSyncConfig.RequestNumber = 1;
    if (HAL_DMAEx_ConfigMuxSync(&hdma_cryp_in, &pSyncConfig) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(crypHandle,hdmain,hdma_cryp_in);

    /* CRYP_OUT Init */
    hdma_cryp_out.Instance = DMA2_Stream2;
    hdma_cryp_out.Init.Request = DMA_REQUEST_CRYP_OUT;
    hdma_cryp_out.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_cryp_out.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_cryp_out.Init.MemInc = DMA_MINC_ENABLE;
    hdma_cryp_out.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_cryp_out.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_cryp_out.Init.Mode = DMA_NORMAL;
    hdma_cryp_out.Init.Priority = DMA_PRIORITY_LOW;
    hdma_cryp_out.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_cryp_out) != HAL_OK)
    {
      Error_Handler();
    }

    pSyncConfig.SyncSignalID = HAL_DMAMUX1_SYNC_EXTI0;
    pSyncConfig.SyncPolarity = HAL_DMAMUX_SYNC_NO_EVENT;
    pSyncConfig.SyncEnable = DISABLE;
    pSyncConfig.EventEnable = DISABLE;
    pSyncConfig.RequestNumber = 1;
    if (HAL_DMAEx_ConfigMuxSync(&hdma_cryp_out, &pSyncConfig) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(crypHandle,hdmaout,hdma_cryp_out);

    /* CRYP interrupt Init */
    HAL_NVIC_SetPriority(CRYP_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(CRYP_IRQn);
  /* USER CODE BEGIN CRYP_MspInit 1 */

  /* USER CODE END CRYP_MspInit 1 */
  }
}

void HAL_CRYP_MspDeInit(CRYP_HandleTypeDef* crypHandle)
{

  if(crypHandle->Instance==CRYP)
  {
  /* USER CODE BEGIN CRYP_MspDeInit 0 */

  /* USER CODE END CRYP_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_CRYP_CLK_DISABLE();

    /* CRYP DMA DeInit */
    HAL_DMA_DeInit(crypHandle->hdmain);
    HAL_DMA_DeInit(crypHandle->hdmaout);

    /* CRYP interrupt Deinit */
    HAL_NVIC_DisableIRQ(CRYP_IRQn);
  /* USER CODE BEGIN CRYP_MspDeInit 1 */

  /* USER CODE END CRYP_MspDeInit 1 */
  }
} 

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
