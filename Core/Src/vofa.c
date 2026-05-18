/**
  ******************************************************************************
  * @file    vofa.c
  * @brief   VOFA+ JustFloat 协议发送（USART1 + 485 + DMA2_Stream0 @ 2 Mbps）
  *
  * 硬件通路：MCU USART1 TX → 485 收发器 → A/B → 上位机的 USB-485 适配器
  * 方向控制：nRTS = PC10 常驻 HIGH，让 485 驱动一直处于发送态
  ******************************************************************************
  */

#include "vofa.h"
#include "usart.h"
#include "main.h"
#include "cmsis_os2.h"
#include <string.h>

/* +inf 小端 = 00 00 80 7F，做 JustFloat 帧分隔符 */
static const uint8_t VOFA_FRAME_TAIL[4] = { 0x00, 0x00, 0x80, 0x7F };

/* DMA 用的发送缓冲（必须常驻，不能放栈上） */
static uint8_t vofa_tx_buf[VOFA_FRAME_MAX_SIZE];

/* DMA TX 完成信号量，由 HAL_UART_TxCpltCallback 释放 */
static osSemaphoreId_t sem_vofa_tx_done = NULL;

void Vofa_Init(void)
{
    /* nRTS 常驻 HIGH（485 驱动一直开启，VOFA+ 数据流不中断） */
    HAL_GPIO_WritePin(USART1_nRTS_GPIO_Port, USART1_nRTS_Pin, GPIO_PIN_SET);

    /* 二值信号量，初始=1（第一次发不等待） */
    sem_vofa_tx_done = osSemaphoreNew(1U, 1U, NULL);
}

int Vofa_IsBusy(void)
{
    return (huart1.gState != HAL_UART_STATE_READY);
}

/* ---- 非阻塞版：丢帧模式 ---- */
int Vofa_SendFloats(const float *data, uint16_t count)
{
    if (data == NULL || count == 0) return -1;
    if (count > VOFA_MAX_CHANNELS)  count = VOFA_MAX_CHANNELS;

    if (Vofa_IsBusy()) return -1;

    uint16_t len = count * 4U;
    memcpy(vofa_tx_buf, data, len);
    memcpy(&vofa_tx_buf[len], VOFA_FRAME_TAIL, 4);
    len += 4U;

    if (HAL_UART_Transmit_DMA(&huart1, vofa_tx_buf, len) != HAL_OK)
    {
        return -1;
    }
    return 0;
}

/* ---- 阻塞版：等上一帧发完再推下一帧，跑满线速 ---- */
int Vofa_SendFloats_Wait(const float *data, uint16_t count)
{
    if (data == NULL || count == 0) return -1;
    if (count > VOFA_MAX_CHANNELS)  count = VOFA_MAX_CHANNELS;
    if (sem_vofa_tx_done == NULL)   return -1;

    /* 等上一次 DMA 发完 */
    osSemaphoreAcquire(sem_vofa_tx_done, osWaitForever);

    uint16_t len = count * 4U;
    memcpy(vofa_tx_buf, data, len);
    memcpy(&vofa_tx_buf[len], VOFA_FRAME_TAIL, 4);
    len += 4U;

    if (HAL_UART_Transmit_DMA(&huart1, vofa_tx_buf, len) != HAL_OK)
    {
        /* 启动失败时要把信号量还回去，不然下次会永远卡住 */
        osSemaphoreRelease(sem_vofa_tx_done);
        return -1;
    }
    return 0;
}

/* ---- HAL UART TX 完成回调：DMA 发完后会进这里 ---- */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1 && sem_vofa_tx_done != NULL)
    {
        osSemaphoreRelease(sem_vofa_tx_done);
		
    }
}
