/**
  ******************************************************************************
  * @file    app_tasks.h
  * @brief   RTX5 任务管理
  *
  * 任务：
  *   mb_task     — Modbus UART 帧处理（从 TIM6 空闲信号量唤醒）
  *   spi3_task   — SPI3 Modbus 帧处理（从 EXTI CS 上升沿信号量唤醒）
  *   app_task    — 应用层预留（周期性心跳）
  *
  * 同步对象：
  *   sem_mb_frame     TIM6 tick 释放，mb_task 等待
  *   sem_spi3_cs_up   EXTI15_10 释放，spi3_task 等待
  *   mutex_regmap     保护 Modbus_RegMap / Modbus_CoilMap
  ******************************************************************************
  */
#ifndef __APP_TASKS_H__
#define __APP_TASKS_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "cmsis_os2.h"

extern osThreadId_t    tid_mb_task;
extern osThreadId_t    tid_spi3_task;
extern osThreadId_t    tid_app_task;

extern osSemaphoreId_t sem_mb_frame;
extern osSemaphoreId_t sem_spi3_cs_up;
extern osMutexId_t     mutex_regmap;

/* 在 osKernelStart() 之前调用：创建信号量/互斥量/任务 */
void App_TasksInit(void);

#ifdef __cplusplus
}
#endif

#endif /* __APP_TASKS_H__ */
