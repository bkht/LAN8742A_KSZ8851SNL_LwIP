/**
  ******************************************************************************
  * File Name          : HASH.c
  * Description        : This file provides code for the configuration
  *                      of the HASH instances.
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
#include "hash.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

HASH_HandleTypeDef hhash;
DMA_HandleTypeDef hdma_hash_in;

/* HASH init function */
void MX_HASH_Init(void)
{

  hhash.Init.DataType = HASH_DATATYPE_32B;
  if (HAL_HASH_Init(&hhash) != HAL_OK)
  {
    Error_Handler();
  }

}

void HAL_HASH_MspInit(HASH_HandleTypeDef* hashHandle)
{

  HAL_DMA_MuxSyncConfigTypeDef pSyncConfig= {0};
  /* USER CODE BEGIN HASH_MspInit 0 */

  /* USER CODE END HASH_MspInit 0 */
    /* HASH clock enable */
    __HAL_RCC_HASH_CLK_ENABLE();
  
    /* HASH DMA Init */
    /* HASH_IN Init */
    hdma_hash_in.Instance = DMA2_Stream3;
    hdma_hash_in.Init.Request = DMA_REQUEST_HASH_IN;
    hdma_hash_in.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_hash_in.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_hash_in.Init.MemInc = DMA_MINC_ENABLE;
    hdma_hash_in.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_hash_in.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_hash_in.Init.Mode = DMA_NORMAL;
    hdma_hash_in.Init.Priority = DMA_PRIORITY_LOW;
    hdma_hash_in.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_hash_in) != HAL_OK)
    {
      Error_Handler();
    }

    pSyncConfig.SyncSignalID = HAL_DMAMUX1_SYNC_EXTI0;
    pSyncConfig.SyncPolarity = HAL_DMAMUX_SYNC_NO_EVENT;
    pSyncConfig.SyncEnable = DISABLE;
    pSyncConfig.EventEnable = DISABLE;
    pSyncConfig.RequestNumber = 1;
    if (HAL_DMAEx_ConfigMuxSync(&hdma_hash_in, &pSyncConfig) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(hashHandle,hdmain,hdma_hash_in);

    /* HASH interrupt Init */
    HAL_NVIC_SetPriority(HASH_RNG_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(HASH_RNG_IRQn);
  /* USER CODE BEGIN HASH_MspInit 1 */

  /* USER CODE END HASH_MspInit 1 */
}

void HAL_HASH_MspDeInit(HASH_HandleTypeDef* hashHandle)
{

  /* USER CODE BEGIN HASH_MspDeInit 0 */

  /* USER CODE END HASH_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_HASH_CLK_DISABLE();

    /* HASH DMA DeInit */
    HAL_DMA_DeInit(hashHandle->hdmain);

    /* HASH interrupt Deinit */
  /* USER CODE BEGIN HASH:HASH_RNG_IRQn disable */
    /**
    * Uncomment the line below to disable the "HASH_RNG_IRQn" interrupt
    * Be aware, disabling shared interrupt may affect other IPs
    */
    /* HAL_NVIC_DisableIRQ(HASH_RNG_IRQn); */
  /* USER CODE END HASH:HASH_RNG_IRQn disable */

  /* USER CODE BEGIN HASH_MspDeInit 1 */

  /* USER CODE END HASH_MspDeInit 1 */
} 

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
