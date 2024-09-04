/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    can.c
  * @brief   This file provides code for the configuration
  *          of the CAN instances.
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
#include "can.h"

/* USER CODE BEGIN 0 */
#include "VescMotor.h"
#include "DJmotor.h"
#include "CANQUEUE.h"
#include "ZDrive.h"
#include "Chassis.h"
/* USER CODE END 0 */

CAN_HandleTypeDef hcan1;
CAN_HandleTypeDef hcan2;

/* CAN1 init function */
void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 7;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_4TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_1TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = ENABLE;
  hcan1.Init.AutoWakeUp = ENABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */
	//DJmotor
  CAN_FilterTypeDef sFilterConfig;
  sFilterConfig.FilterActivation = CAN_FILTER_ENABLE;
  sFilterConfig.FilterMode = CAN_FILTERMODE_IDLIST;
  sFilterConfig.FilterScale = CAN_FILTERSCALE_16BIT;
  sFilterConfig.FilterBank = 1;
	sFilterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;
  sFilterConfig.FilterIdHigh = 0x201 << 5;
  sFilterConfig.FilterIdLow = 0x202 << 5;
  sFilterConfig.FilterMaskIdHigh = 0x203 <<5;
  sFilterConfig.FilterMaskIdLow = 0x204 << 5;
	sFilterConfig.SlaveStartFilterBank = 14;
  if(HAL_CAN_ConfigFilter(&hcan1,&sFilterConfig)!=HAL_OK)
  Error_Handler();

  sFilterConfig.FilterActivation = CAN_FILTER_ENABLE;
  sFilterConfig.FilterMode = CAN_FILTERMODE_IDLIST;
  sFilterConfig.FilterScale = CAN_FILTERSCALE_16BIT;
  sFilterConfig.FilterBank = 2;
  sFilterConfig.FilterFIFOAssignment = CAN_FilterFIFO0;
  sFilterConfig.FilterIdHigh = 0x205 <<5;
  sFilterConfig.FilterIdLow = 0x206 <<5;
  sFilterConfig.FilterMaskIdHigh = 0x207 <<5;
  sFilterConfig.FilterIdLow = 0x208 <<5;
	sFilterConfig.SlaveStartFilterBank = 14;
  if(HAL_CAN_ConfigFilter(&hcan1,&sFilterConfig)!=HAL_OK)
    Error_Handler();
  if(HAL_CAN_Start(&hcan1)!= HAL_OK ) 
    Error_Handler();
  if(HAL_CAN_ActivateNotification(&hcan1,CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
    Error_Handler();
  /* USER CODE END CAN1_Init 2 */

}
/* CAN2 init function */
void MX_CAN2_Init(void)
{

  /* USER CODE BEGIN CAN2_Init 0 */

  /* USER CODE END CAN2_Init 0 */

  /* USER CODE BEGIN CAN2_Init 1 */

  /* USER CODE END CAN2_Init 1 */
  hcan2.Instance = CAN2;
  hcan2.Init.Prescaler = 7;
  hcan2.Init.Mode = CAN_MODE_NORMAL;
  hcan2.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan2.Init.TimeSeg1 = CAN_BS1_4TQ;
  hcan2.Init.TimeSeg2 = CAN_BS2_1TQ;
  hcan2.Init.TimeTriggeredMode = DISABLE;
  hcan2.Init.AutoBusOff = ENABLE;
  hcan2.Init.AutoWakeUp = ENABLE;
  hcan2.Init.AutoRetransmission = DISABLE;
  hcan2.Init.ReceiveFifoLocked = DISABLE;
  hcan2.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN2_Init 2 */
  //vescmotor
  CAN_FilterTypeDef sFilterConfig;
  sFilterConfig.FilterActivation = CAN_FILTER_ENABLE;
  sFilterConfig.FilterBank = 15;
  sFilterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO1;
  sFilterConfig.FilterMode =CAN_FILTERMODE_IDMASK;
  sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
  sFilterConfig.FilterIdHigh = ((((uint32_t)CAN_PACKET_STATUS_4 << 8) << 3) & 0xffff0000) >> 16;
  sFilterConfig.FilterIdLow = ((((uint32_t)CAN_PACKET_STATUS_4 << 8) << 3) & 0xffff);
  sFilterConfig.FilterMaskIdHigh = (0xffffff00 << 3) >> 16;
  sFilterConfig.FilterMaskIdLow = (0xffffff00 << 3) & 0xffff;
  sFilterConfig.SlaveStartFilterBank = 14;
  if(HAL_CAN_ConfigFilter(&hcan2,&sFilterConfig)!=HAL_OK)
    Error_Handler();

  sFilterConfig.FilterActivation = CAN_FILTER_ENABLE;
  sFilterConfig.FilterBank = 16;
  sFilterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO1;
  sFilterConfig.FilterMode =CAN_FILTERMODE_IDMASK;
  sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
  sFilterConfig.FilterIdHigh = ((((uint32_t)CAN_PACKET_STATUS << 8) << 3) & 0xffff0000) >> 16;
  sFilterConfig.FilterIdLow = ((((uint32_t)CAN_PACKET_STATUS << 8) << 3) & 0xffff);
  sFilterConfig.FilterMaskIdHigh = (0xffffff00 << 3) >> 16;
  sFilterConfig.FilterMaskIdLow = (0xffffff00 << 3) & 0xffff;
  sFilterConfig.SlaveStartFilterBank = 14;
  if(HAL_CAN_ConfigFilter(&hcan2,&sFilterConfig)!=HAL_OK)
    Error_Handler();

  //Zdrive
  sFilterConfig.FilterActivation =CAN_FILTER_ENABLE;
  sFilterConfig.FilterBank = 22;
  sFilterConfig.FilterMode = CAN_FILTERMODE_IDLIST;
  sFilterConfig.FilterScale = CAN_FILTERSCALE_16BIT;
  sFilterConfig.FilterFIFOAssignment = CAN_FilterFIFO1;
  sFilterConfig.FilterIdHigh = (0x001) << 5;
  sFilterConfig.FilterIdLow = (0x002) << 5;
  sFilterConfig.FilterMaskIdHigh = (0x003) << 5;
  sFilterConfig.FilterMaskIdLow = (0x004) << 5;
  sFilterConfig.SlaveStartFilterBank = 14;
  if(HAL_CAN_ConfigFilter(&hcan2,&sFilterConfig)!=HAL_OK)
    Error_Handler();
  
  sFilterConfig.FilterActivation =CAN_FILTER_ENABLE;
  sFilterConfig.FilterBank = 23;
  sFilterConfig.FilterMode = CAN_FILTERMODE_IDLIST;
  sFilterConfig.FilterScale = CAN_FILTERSCALE_16BIT;
  sFilterConfig.FilterFIFOAssignment = CAN_FilterFIFO1;
  sFilterConfig.FilterIdHigh = (0x005) << 5;
  sFilterConfig.FilterIdLow = (0x006) << 5;
  sFilterConfig.FilterMaskIdHigh = (0x007) << 5;
  sFilterConfig.FilterMaskIdLow = (0x008) << 5;
  sFilterConfig.SlaveStartFilterBank = 14;
  if(HAL_CAN_ConfigFilter(&hcan2,&sFilterConfig)!=HAL_OK)
    Error_Handler();

	//主机通讯
	sFilterConfig.FilterActivation = CAN_FILTER_ENABLE;
  sFilterConfig.FilterBank = 24;
  sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
  sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
  sFilterConfig.FilterFIFOAssignment = CAN_FilterFIFO0;
  sFilterConfig.FilterIdHigh = ((CAN_MASTER_ID << 3) & 0xffff0000) >> 16;
  sFilterConfig.FilterIdLow = ((CAN_MASTER_ID << 3) & 0xffff);
  sFilterConfig.FilterMaskIdHigh = (CAN_MASTER_MASK << 3) >> 16;
  sFilterConfig.FilterMaskIdLow = ((CAN_MASTER_MASK << 3)) & 0xffff;
  sFilterConfig.SlaveStartFilterBank = 14;
  if(HAL_CAN_ConfigFilter(&hcan2,&sFilterConfig)!=HAL_OK)
    Error_Handler();
	
  if(HAL_CAN_Start(&hcan2)!= HAL_OK ) 
    Error_Handler();
  if(HAL_CAN_ActivateNotification(&hcan2,CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
    Error_Handler();
  if(HAL_CAN_ActivateNotification(&hcan2,CAN_IT_RX_FIFO1_MSG_PENDING) != HAL_OK)
    Error_Handler();
  /* USER CODE END CAN2_Init 2 */

}

static uint32_t HAL_RCC_CAN1_CLK_ENABLED=0;

void HAL_CAN_MspInit(CAN_HandleTypeDef* canHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspInit 0 */

  /* USER CODE END CAN1_MspInit 0 */
    /* CAN1 clock enable */
    HAL_RCC_CAN1_CLK_ENABLED++;
    if(HAL_RCC_CAN1_CLK_ENABLED==1){
      __HAL_RCC_CAN1_CLK_ENABLE();
    }

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**CAN1 GPIO Configuration
    PA11     ------> CAN1_RX
    PA12     ------> CAN1_TX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF9_CAN1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* CAN1 interrupt Init */
    HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
    HAL_NVIC_SetPriority(CAN1_RX1_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(CAN1_RX1_IRQn);
  /* USER CODE BEGIN CAN1_MspInit 1 */

  /* USER CODE END CAN1_MspInit 1 */
  }
  else if(canHandle->Instance==CAN2)
  {
  /* USER CODE BEGIN CAN2_MspInit 0 */

  /* USER CODE END CAN2_MspInit 0 */
    /* CAN2 clock enable */
    __HAL_RCC_CAN2_CLK_ENABLE();
    HAL_RCC_CAN1_CLK_ENABLED++;
    if(HAL_RCC_CAN1_CLK_ENABLED==1){
      __HAL_RCC_CAN1_CLK_ENABLE();
    }

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**CAN2 GPIO Configuration
    PB5     ------> CAN2_RX
    PB6     ------> CAN2_TX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_6;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF9_CAN2;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* CAN2 interrupt Init */
    HAL_NVIC_SetPriority(CAN2_RX0_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(CAN2_RX0_IRQn);
    HAL_NVIC_SetPriority(CAN2_RX1_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(CAN2_RX1_IRQn);
  /* USER CODE BEGIN CAN2_MspInit 1 */
  //记得使能
  /* USER CODE END CAN2_MspInit 1 */
  }
}

void HAL_CAN_MspDeInit(CAN_HandleTypeDef* canHandle)
{

  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspDeInit 0 */

  /* USER CODE END CAN1_MspDeInit 0 */
    /* Peripheral clock disable */
    HAL_RCC_CAN1_CLK_ENABLED--;
    if(HAL_RCC_CAN1_CLK_ENABLED==0){
      __HAL_RCC_CAN1_CLK_DISABLE();
    }

    /**CAN1 GPIO Configuration
    PA11     ------> CAN1_RX
    PA12     ------> CAN1_TX
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_11|GPIO_PIN_12);

    /* CAN1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(CAN1_RX0_IRQn);
    HAL_NVIC_DisableIRQ(CAN1_RX1_IRQn);
  /* USER CODE BEGIN CAN1_MspDeInit 1 */

  /* USER CODE END CAN1_MspDeInit 1 */
  }
  else if(canHandle->Instance==CAN2)
  {
  /* USER CODE BEGIN CAN2_MspDeInit 0 */

  /* USER CODE END CAN2_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_CAN2_CLK_DISABLE();
    HAL_RCC_CAN1_CLK_ENABLED--;
    if(HAL_RCC_CAN1_CLK_ENABLED==0){
      __HAL_RCC_CAN1_CLK_DISABLE();
    }

    /**CAN2 GPIO Configuration
    PB5     ------> CAN2_RX
    PB6     ------> CAN2_TX
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_5|GPIO_PIN_6);

    /* CAN2 interrupt Deinit */
    HAL_NVIC_DisableIRQ(CAN2_RX0_IRQn);
    HAL_NVIC_DisableIRQ(CAN2_RX1_IRQn);
  /* USER CODE BEGIN CAN2_MspDeInit 1 */

  /* USER CODE END CAN2_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) 
{   
    CAN_RxHeaderTypeDef Rxheader;
    uint8_t Rxdata[8];
    
    if(hcan->Instance == CAN1)
    {
        HAL_CAN_GetRxMessage(hcan,CAN_RX_FIFO0,&Rxheader,Rxdata);
        DJReceiveData_CAN1(Rxheader,Rxdata);
    }
    else if(hcan->Instance == CAN2)
    {
        HAL_CAN_GetRxMessage(hcan,CAN_RX_FIFO0,&Rxheader,Rxdata);
        ChassisFunc(Rxheader,Rxdata);
    }
    
}
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef Rxheader;
    uint8_t Rxdata[8];
    if(hcan->Instance == CAN2){
			HAL_CAN_GetRxMessage(hcan,CAN_RX_FIFO1,&Rxheader,Rxdata);
      if(Rxheader.IDE == CAN_ID_EXT){ 
        VescReceiveData_CAN2(Rxheader,Rxdata);
      }
      else if(Rxheader.IDE == CAN_ID_STD)
        ZdriveReceive(Rxheader,Rxdata);
    }

}
/* USER CODE END 1 */
