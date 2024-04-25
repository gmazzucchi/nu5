/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    dac.c
  * @brief   This file provides code for the configuration
  *          of the DAC instances.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "dac.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

DAC_HandleTypeDef hdac1;
DMA_NodeTypeDef Node_GPDMA1_Channel10;
DMA_QListTypeDef List_GPDMA1_Channel10;
DMA_HandleTypeDef handle_GPDMA1_Channel10;

/* DAC1 init function */
void MX_DAC1_Init(void) {
    /* USER CODE BEGIN DAC1_Init 0 */

    /* USER CODE END DAC1_Init 0 */

    DAC_ChannelConfTypeDef sConfig                = {0};
    DAC_AutonomousModeConfTypeDef sAutonomousMode = {0};

    /* USER CODE BEGIN DAC1_Init 1 */

    /* USER CODE END DAC1_Init 1 */

    /** DAC Initialization
  */
    hdac1.Instance = DAC1;
    if (HAL_DAC_Init(&hdac1) != HAL_OK) {
        Error_Handler();
    }

    /** DAC channel OUT1 config
  */
    sConfig.DAC_HighFrequency           = DAC_HIGH_FREQUENCY_INTERFACE_MODE_DISABLE;
    sConfig.DAC_DMADoubleDataMode       = DISABLE;
    sConfig.DAC_SignedFormat            = DISABLE;
    sConfig.DAC_SampleAndHold           = DAC_SAMPLEANDHOLD_DISABLE;
    sConfig.DAC_Trigger                 = DAC_TRIGGER_NONE;
    sConfig.DAC_OutputBuffer            = DAC_OUTPUTBUFFER_DISABLE;
    sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_EXTERNAL;
    sConfig.DAC_UserTrimming            = DAC_TRIMMING_FACTORY;
    if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_1) != HAL_OK) {
        Error_Handler();
    }

    /** Configure Autonomous Mode
  */
    sAutonomousMode.AutonomousModeState = DAC_AUTONOMOUS_MODE_DISABLE;
    if (HAL_DACEx_SetConfigAutonomousMode(&hdac1, &sAutonomousMode) != HAL_OK) {
        Error_Handler();
    }
    /* USER CODE BEGIN DAC1_Init 2 */

    /* USER CODE END DAC1_Init 2 */
}

void HAL_DAC_MspInit(DAC_HandleTypeDef *dacHandle) {
    GPIO_InitTypeDef GPIO_InitStruct       = {0};
    DMA_NodeConfTypeDef NodeConfig         = {0};
    RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};
    if (dacHandle->Instance == DAC1) {
        /* USER CODE BEGIN DAC1_MspInit 0 */

        /* USER CODE END DAC1_MspInit 0 */

        /** Initializes the peripherals clock
  */
        PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADCDAC | RCC_PERIPHCLK_DAC1;
        PeriphClkInit.AdcDacClockSelection = RCC_ADCDACCLKSOURCE_HSI;
        PeriphClkInit.Dac1ClockSelection   = RCC_DAC1CLKSOURCE_LSI;
        if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
            Error_Handler();
        }

        /* DAC1 clock enable */
        __HAL_RCC_DAC1_CLK_ENABLE();

        __HAL_RCC_GPIOA_CLK_ENABLE();
        /**DAC1 GPIO Configuration
    PA4     ------> DAC1_OUT1
    */
        GPIO_InitStruct.Pin  = GPIO_PIN_4;
        GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

        /* DAC1 DMA Init */
        /* GPDMA1_REQUEST_DAC1_CH1 Init */
        NodeConfig.NodeType                         = DMA_GPDMA_LINEAR_NODE;
        NodeConfig.Init.Request                     = GPDMA1_REQUEST_DAC1_CH1;
        NodeConfig.Init.BlkHWRequest                = DMA_BREQ_SINGLE_BURST;
        NodeConfig.Init.Direction                   = DMA_MEMORY_TO_PERIPH;
        NodeConfig.Init.SrcInc                      = DMA_SINC_INCREMENTED;
        NodeConfig.Init.DestInc                     = DMA_DINC_FIXED;
        NodeConfig.Init.SrcDataWidth                = DMA_SRC_DATAWIDTH_HALFWORD;
        NodeConfig.Init.DestDataWidth               = DMA_DEST_DATAWIDTH_HALFWORD;
        NodeConfig.Init.SrcBurstLength              = 1;
        NodeConfig.Init.DestBurstLength             = 1;
        NodeConfig.Init.TransferAllocatedPort       = DMA_SRC_ALLOCATED_PORT0 | DMA_DEST_ALLOCATED_PORT0;
        NodeConfig.Init.TransferEventMode           = DMA_TCEM_BLOCK_TRANSFER;
        NodeConfig.Init.Mode                        = DMA_NORMAL;
        NodeConfig.TriggerConfig.TriggerPolarity    = DMA_TRIG_POLARITY_MASKED;
        NodeConfig.DataHandlingConfig.DataExchange  = DMA_EXCHANGE_NONE;
        NodeConfig.DataHandlingConfig.DataAlignment = DMA_DATA_RIGHTALIGN_ZEROPADDED;
        if (HAL_DMAEx_List_BuildNode(&NodeConfig, &Node_GPDMA1_Channel10) != HAL_OK) {
            Error_Handler();
        }

        if (HAL_DMAEx_List_InsertNode(&List_GPDMA1_Channel10, NULL, &Node_GPDMA1_Channel10) != HAL_OK) {
            Error_Handler();
        }

        if (HAL_DMAEx_List_SetCircularMode(&List_GPDMA1_Channel10) != HAL_OK) {
            Error_Handler();
        }

        handle_GPDMA1_Channel10.Instance                         = GPDMA1_Channel10;
        handle_GPDMA1_Channel10.InitLinkedList.Priority          = DMA_HIGH_PRIORITY;
        handle_GPDMA1_Channel10.InitLinkedList.LinkStepMode      = DMA_LSM_FULL_EXECUTION;
        handle_GPDMA1_Channel10.InitLinkedList.LinkAllocatedPort = DMA_LINK_ALLOCATED_PORT0;
        handle_GPDMA1_Channel10.InitLinkedList.TransferEventMode = DMA_TCEM_BLOCK_TRANSFER;
        handle_GPDMA1_Channel10.InitLinkedList.LinkedListMode    = DMA_LINKEDLIST_CIRCULAR;
        if (HAL_DMAEx_List_Init(&handle_GPDMA1_Channel10) != HAL_OK) {
            Error_Handler();
        }

        if (HAL_DMAEx_List_LinkQ(&handle_GPDMA1_Channel10, &List_GPDMA1_Channel10) != HAL_OK) {
            Error_Handler();
        }

        __HAL_LINKDMA(dacHandle, DMA_Handle1, handle_GPDMA1_Channel10);

        if (HAL_DMA_ConfigChannelAttributes(&handle_GPDMA1_Channel10, DMA_CHANNEL_NPRIV) != HAL_OK) {
            Error_Handler();
        }

        /* USER CODE BEGIN DAC1_MspInit 1 */

        /* USER CODE END DAC1_MspInit 1 */
    }
}

void HAL_DAC_MspDeInit(DAC_HandleTypeDef *dacHandle) {
    if (dacHandle->Instance == DAC1) {
        /* USER CODE BEGIN DAC1_MspDeInit 0 */

        /* USER CODE END DAC1_MspDeInit 0 */
        /* Peripheral clock disable */
        __HAL_RCC_DAC1_CLK_DISABLE();

        /**DAC1 GPIO Configuration
    PA4     ------> DAC1_OUT1
    */
        HAL_GPIO_DeInit(GPIOA, GPIO_PIN_4);

        /* DAC1 DMA DeInit */
        HAL_DMA_DeInit(dacHandle->DMA_Handle1);
        /* USER CODE BEGIN DAC1_MspDeInit 1 */

        /* USER CODE END DAC1_MspDeInit 1 */
    }
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
