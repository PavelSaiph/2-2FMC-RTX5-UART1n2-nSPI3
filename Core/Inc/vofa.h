/**
  ******************************************************************************
  * @file    vofa.h
  * @brief   VOFA+ JustFloat 协议封装（USART6 DMA 发送）
  *
  * 帧格式：
  *   [float1_LE 4B][float2_LE 4B]...[floatN_LE 4B][00 00 80 7F]
  *   末尾 4 字节是 +inf 小端 IEEE754，做帧分隔符
  *
  * 用法：
  *   float ch[3] = {v1, v2, v3};
  *   Vofa_SendFloats(ch, 3);   // 非阻塞，DMA 后台发
  ******************************************************************************
  */
#ifndef __VOFA_H__
#define __VOFA_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/* 支持的最大通道数（每通道 4 字节 + 尾 4 字节） */
#define VOFA_MAX_CHANNELS    32
#define VOFA_FRAME_MAX_SIZE  (VOFA_MAX_CHANNELS * 4 + 4)

/* 初始化（调一次，要在 osKernelStart 之前，信号量才能建好）*/
void Vofa_Init(void);

/* 非阻塞发送：上一帧未完成则直接丢弃本帧，返回 -1 */
int Vofa_SendFloats(const float *data, uint16_t count);

/* 阻塞发送：等上一次 DMA 完成再启动本次，保证一帧不丢
 *   用在 VOFA+ 高速流里：速率由 DMA 完成速度决定（约 10 kHz @ 3ch）
 */
int Vofa_SendFloats_Wait(const float *data, uint16_t count);

/* 查询 DMA 是否正在发送 */
int Vofa_IsBusy(void);

#ifdef __cplusplus
}
#endif

#endif /* __VOFA_H__ */
