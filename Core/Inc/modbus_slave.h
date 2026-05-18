/**
  ******************************************************************************
  * @file    modbus_slave.h
  * @brief   Modbus RTU slave (USART2 RS485) - Armfly style
  *
  * 帧同步：3.5 字符空闲时间判帧结束，用 TIM6 作 1ms tick
  * 支持功能码：0x03 / 0x06 / 0x10
  * CRC：标准 Modbus (多项式 0xA001)
  * 从机地址：MODBUS_SLAVE_ADDR（默认 0x01）
  * 寄存器区：复用 main.c 里定义的 Modbus_RegMap[MAX_REG_ADDR]
  ******************************************************************************
  */

#ifndef __MODBUS_SLAVE_H__
#define __MODBUS_SLAVE_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/* -------- 用户可调 -------- */
#define MODBUS_SLAVE_ADDR         0x01        /* 本机从机地址                         */
#define MODBUS_FRAME_SILENCE_MS   2           /* 帧结束判定：连续空闲ms数             */
#define MODBUS_RX_BUF_SIZE        256
#define MODBUS_TX_BUF_SIZE        256

/* -------- 寄存器区（main.c 定义，这里 extern） -------- */
#define MAX_REG_ADDR              10000
extern uint16_t Modbus_RegMap[MAX_REG_ADDR];

/* -------- 线圈区（用位图存储，2000 个线圈 = 250 字节） -------- */
#define MAX_COIL_ADDR             2000
#define MAX_COIL_BYTES            ((MAX_COIL_ADDR + 7) / 8)
extern uint8_t Modbus_CoilMap[MAX_COIL_BYTES];

/* -------- 功能码 -------- */
#define MB_FC_READ_COILS          0x01
#define MB_FC_READ_DISC_INPUTS    0x02
#define MB_FC_READ_HOLDING_REG    0x03
#define MB_FC_READ_INPUT_REG      0x04
#define MB_FC_WRITE_SINGLE_COIL   0x05
#define MB_FC_WRITE_SINGLE_REG    0x06
#define MB_FC_WRITE_MULTI_COILS   0x0F
#define MB_FC_WRITE_MULTI_REG     0x10

/* -------- 异常码 -------- */
#define MB_EX_ILLEGAL_FUNCTION    0x01
#define MB_EX_ILLEGAL_ADDRESS     0x02
#define MB_EX_ILLEGAL_VALUE       0x03
#define MB_EX_SLAVE_FAILURE       0x04

/* -------- API -------- */
void Modbus_Init(void);              /* 初始化：TIM6 + USART2 接收中断 */
void Modbus_RxByte(uint8_t byte);    /* UART RX 回调里调用，喂一个字节 */
void Modbus_TimerTick_1ms(void);     /* TIM6 中断里调用，每 1ms 一次   */
void Modbus_Poll(void);              /* 主循环里调用，处理收到的帧     */
void Modbus_ResetRxState(void);      /* UART 错误回调里调用，丢弃半成品帧 */

#ifdef __cplusplus
}
#endif

#endif /* __MODBUS_SLAVE_H__ */
