/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.c
  * @brief   This file provides code for the configuration
  *          of the USART instances.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "usart.h"

/* USER CODE BEGIN 0 */
#include "modbus_slave.h"

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
DMA_HandleTypeDef  hdma_usart1_tx;   /* VOFA+ DMA 发送句柄 */
/* USER CODE END 0 */

/* USART1 init function */

void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  /* USART1 改作 VOFA+ JustFloat 专用 (2 Mbps, TX-only DMA)
   * 485 收发器仍在此 UART，nRTS(PC10) 常驻 HIGH 让 485 驱动一直开启
   */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 2000000;            /* 2 Mbps 高速发送 */
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX;           /* 只发，不收 */
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}
/* USART2 init function */

void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
//  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  // 开启高级特性初始化
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_SWAP_INIT;
  // 使能 TX 和 RX 引脚交叉
  huart2.AdvancedInit.Swap = UART_ADVFEATURE_SWAP_ENABLE;
  
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}
void HAL_UART_MspInit(UART_HandleTypeDef* uartHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};
  if(uartHandle->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspInit 0 */

  /* USER CODE END USART1_MspInit 0 */

  /** Initializes the peripherals clock
  */
    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART1;
    PeriphClkInitStruct.Usart16ClockSelection = RCC_USART16CLKSOURCE_D2PCLK2;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
    {
      Error_Handler();
    }

    /* USART1 clock enable */
    __HAL_RCC_USART1_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_DMA2_CLK_ENABLE();

    /**USART1 GPIO Configuration
    PA9     ------> USART1_TX
    PA10     ------> USART1_RX (已禁用，Mode=TX 后不会用)
    2 Mbps 必须用 VERY_HIGH 速度，避免边沿塌陷 */
    GPIO_InitStruct.Pin = GPIO_PIN_9|GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* USART1_TX DMA —— DMA2_Stream0 */
    hdma_usart1_tx.Instance                 = DMA2_Stream0;
    hdma_usart1_tx.Init.Request             = DMA_REQUEST_USART1_TX;
    hdma_usart1_tx.Init.Direction           = DMA_MEMORY_TO_PERIPH;
    hdma_usart1_tx.Init.PeriphInc           = DMA_PINC_DISABLE;
    hdma_usart1_tx.Init.MemInc              = DMA_MINC_ENABLE;
    hdma_usart1_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart1_tx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
    hdma_usart1_tx.Init.Mode                = DMA_NORMAL;
    hdma_usart1_tx.Init.Priority            = DMA_PRIORITY_HIGH;
    hdma_usart1_tx.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_usart1_tx) != HAL_OK)
    {
      Error_Handler();
    }
    __HAL_LINKDMA(uartHandle, hdmatx, hdma_usart1_tx);

    HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

    /* USART1 中断（DMA TC 会经过 USART 中断走 HAL 回调）*/
    HAL_NVIC_SetPriority(USART1_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
  /* USER CODE BEGIN USART1_MspInit 1 */

  /* USER CODE END USART1_MspInit 1 */
  }
  else if(uartHandle->Instance==USART2)
  {
  /* USER CODE BEGIN USART2_MspInit 0 */

  /* USER CODE END USART2_MspInit 0 */

  /** Initializes the peripherals clock
  */
    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART2;
    PeriphClkInitStruct.Usart234578ClockSelection = RCC_USART234578CLKSOURCE_D2PCLK1;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
    {
      Error_Handler();
    }

    /* USART2 clock enable */
    __HAL_RCC_USART2_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**USART2 GPIO Configuration
    PA2     ------> USART2_TX
    PA3     ------> USART2_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* USART2 interrupt Init */
    HAL_NVIC_SetPriority(USART2_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USART2_IRQn);
  /* USER CODE BEGIN USART2_MspInit 1 */

  /* USER CODE END USART2_MspInit 1 */
  }
}

void HAL_UART_MspDeInit(UART_HandleTypeDef* uartHandle)
{

  if(uartHandle->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspDeInit 0 */

  /* USER CODE END USART1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART1_CLK_DISABLE();

    /**USART1 GPIO Configuration
    PA9     ------> USART1_TX
    PA10     ------> USART1_RX
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_9|GPIO_PIN_10);

    /* USART1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USART1_IRQn);
  /* USER CODE BEGIN USART1_MspDeInit 1 */

  /* USER CODE END USART1_MspDeInit 1 */
  }
  else if(uartHandle->Instance==USART2)
  {
  /* USER CODE BEGIN USART2_MspDeInit 0 */

  /* USER CODE END USART2_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART2_CLK_DISABLE();

    /**USART2 GPIO Configuration
    PA2     ------> USART2_TX
    PA3     ------> USART2_RX
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_2|GPIO_PIN_3);

    /* USART2 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USART2_IRQn);
  /* USER CODE BEGIN USART2_MspDeInit 1 */

  /* USER CODE END USART2_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */

/*
功能：发送Size个字节的数据
参数：	@huart：usart句柄，可选值：huart1，huart2
		@pData：指向需要发送数据的寄存器的指针变量
		@Size： 需要发送数据的字节数
*/
void MyUsart_SendData(UART_HandleTypeDef* huart, uint8_t* pData, uint32_t Size)
{
//切换为发送模式（射极跟随器结构：nRTS HIGH → 光耦导通 → RE2 HIGH → DE=1 发送）
	if(huart->Instance == USART1)
	{
		HAL_GPIO_WritePin(USART1_nRTS_GPIO_Port, USART1_nRTS_Pin, GPIO_PIN_SET);
	}
	else if(huart->Instance == USART2)
	{
		HAL_GPIO_WritePin(USART2_nRTS_GPIO_Port, USART2_nRTS_Pin, GPIO_PIN_SET);
	}

	HAL_UART_Transmit(huart, pData, Size, 50);  //TX 超时 50ms（Modbus 256B 最多 22ms @115200）

	//等 TC 标志置位，加 50ms 超时，防止总线死锁
	{
		uint32_t tc_start = HAL_GetTick();
		while(__HAL_UART_GET_FLAG(huart, UART_FLAG_TC) == RESET)
		{
			if ((HAL_GetTick() - tc_start) > 50) break;
		}
	}

//切换为接受模式
	if(huart->Instance == USART1)
	{
		HAL_GPIO_WritePin(USART1_nRTS_GPIO_Port, USART1_nRTS_Pin, GPIO_PIN_RESET);
	}
	else if(huart->Instance == USART2)
	{
		HAL_GPIO_WritePin(USART2_nRTS_GPIO_Port, USART2_nRTS_Pin, GPIO_PIN_RESET);
	}
}
/*************************接收*********************/ 
/*使用时先在初始化调用MyUsart_EnableReceiveByte 
然后在死循环中调用MyUsart_ReceiveByte */  
static uint8_t RxBuffer1; 
volatile static uint8_t MyUsart_RxFlag1=0;  

static uint8_t RxBuffer2; 
volatile static uint8_t MyUsart_RxFlag2=0;  
/* 
功能：使能接受一个字节的中断 
描述：只需要在初始化时调用一次，接收的状态就一直保持 
参数：	@huart：包含指向USARTx的句柄huartx 
*/ 
void MyUsart_EnableReceiveByte(UART_HandleTypeDef* huart) 
{ 	
	
	if(huart->Instance == USART1)
	{
		MyUsart_RxFlag1 = 0;
		HAL_GPIO_WritePin(USART1_nRTS_GPIO_Port, USART1_nRTS_Pin, GPIO_PIN_RESET);  //接收态：拉低
		HAL_UART_Receive_IT(huart, &RxBuffer1, 1);
	}
	else if(huart->Instance == USART2)
	{
		MyUsart_RxFlag2 = 0;
		HAL_GPIO_WritePin(USART2_nRTS_GPIO_Port, USART2_nRTS_Pin, GPIO_PIN_RESET);  //接收态：拉低
		HAL_UART_Receive_IT(huart, &RxBuffer2, 1);
	}
	
}  

/* 
功能：等待接收一个字节的数据 
描述：在初始化调用MyUsart_EnableReceiveByte后，可以在函数主体死循环中调用这个函数用来等待字节 
返回值为1或者0,1代表接收成功 ，且数据存在pData里
参数：	@huart: 句柄
		@pData：用于接收数据的指针 
*/ 
uint8_t MyUsart_ReceiveByte(UART_HandleTypeDef* huart, uint8_t* pData) 
{
	// --- 检查 USART1 ---
    if(huart->Instance == USART1)
    {
        if(MyUsart_RxFlag1 == 1)
        {
            *pData = RxBuffer1; // 取 buffer1
            MyUsart_RxFlag1 = 0;        // 清 flag1
            return 1;
        }
    }
    // --- 检查 USART2 ---
    else if(huart->Instance == USART2)
    {
        if(MyUsart_RxFlag2 == 1)
        {
            *pData = RxBuffer2; // 取 buffer2
            MyUsart_RxFlag2 = 0;        // 清 flag2
            return 1;
        }
    }
    
    return 0; // 该串口没有新数据
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef* huart) 
{ 	
	if(huart->Instance == USART1)
	{         
		// RxBuffer 里现在有数据了，可以在这里处理它         
		// 例如：MyProcessData(RxBuffer);         
		MyUsart_RxFlag1=1; 		 		
		HAL_UART_Receive_IT(huart, &RxBuffer1, 1);     
	} 
	else if(huart->Instance == USART2)
	{
		Modbus_RxByte(RxBuffer2);          /* 喂给 Modbus 帧缓冲 */
		MyUsart_RxFlag2=1;
		HAL_UART_Receive_IT(huart, &RxBuffer2, 1);
	}
}

/*
功能：UART 错误回调（在 UART ISR 里调用）
描述：处理 ORE（溢出）/ FE（帧错）/ NE（噪声）/ PE（校验）错误，
      清错误标志位、复位 HAL 状态、丢弃当前 Modbus 半成品帧、重新武装接收
*/
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    /* 清所有错误标志：ORE、FE、NE、PE */
    __HAL_UART_CLEAR_FLAG(huart, UART_CLEAR_OREF | UART_CLEAR_FEF |
                                 UART_CLEAR_NEF  | UART_CLEAR_PEF);

    /* 手动复位 HAL 接收状态，保证下面 Receive_IT 能进去 */
    huart->RxState   = HAL_UART_STATE_READY;
    huart->ErrorCode = HAL_UART_ERROR_NONE;

    if (huart->Instance == USART1)
    {
        /* USART1 改作 VOFA+ TX-only，这里不重启 RX */
    }
    else if (huart->Instance == USART2)
    {
        /* 丢弃当前半截帧（可能被噪声污染） */
        Modbus_ResetRxState();
        MyUsart_RxFlag2 = 0;
        HAL_UART_Receive_IT(huart, &RxBuffer2, 1);
    }
} 
/********************************************************/
/* USER CODE END 1 */
