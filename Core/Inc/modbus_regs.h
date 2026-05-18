/**
  ******************************************************************************
  * @file    modbus_regs.h
  * @brief   自主中频电源控制板 ↔ 通讯卡 交互数据（Modbus 保持寄存器映射表）
  *
  * 对应文档：自主中频电源控制板与通讯卡交互数据列表 V1.0
  *
  * 用法：
  *   1) main.c 里 Modbus_Init() 之前调用 Modbus_LoadDefaults()，把表里
  *      所有条目的出厂值批量写入 Modbus_RegMap。
  *   2) Modbus 写寄存器 (0x06/0x10) 时先调 Modbus_ValidateRegWrite：
  *      - 工厂只读区 → 拒绝，返回异常码 0x02 (非法地址)
  *      - 超出 [min, max] → 拒绝，返回异常码 0x03 (非法数据)
  *   3) 应用层直接用宏名读寄存器：Modbus_RegMap[REG_OUTPUT_ONOFF]
  ******************************************************************************
  */

#ifndef __MODBUS_REGS_H__
#define __MODBUS_REGS_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "modbus_slave.h"

/* ========== 机器出厂限值（根据实际机型调整） ========== */
#define UNIT_POWER_LIMIT_W         15000   /* 15 kW（AP 15KW 机型）    */
#define UNIT_VOLTAGE_LIMIT_V       600
#define UNIT_CURRENT_LIMIT_01A     1500    /* 150.0 A, 0.1A 缩放       */

/* 32-bit 数值拆分为两个寄存器（高字在前，Modbus 常规做法） */
#define HI16(x)                    ((uint16_t)(((uint32_t)(x) >> 16) & 0xFFFFU))
#define LO16(x)                    ((uint16_t)( (uint32_t)(x)        & 0xFFFFU))

/* ==================== 寄存器地址（按交互数据列表） ==================== */

/* ----- System / Basic (工厂只读) ----- */
#define REG_UNIT_TYPE              0x0000   /* 10 regs = 20B ASCII 电源型号  */
#define REG_UNIT_NUMBER            0x000A   /* 5 regs = 10B 生产序列号        */
#define REG_FPGA1_VERSION          0x000F   /* 5 regs = 10B                  */
#define REG_FPGA2_VERSION          0x0014   /* 5 regs = 10B                  */
#define REG_ARM1_VERSION           0x0019   /* 5 regs = 10B                  */
#define REG_ARM2_VERSION           0x001E   /* 5 regs = 10B                  */
#define REG_UNIT_POWER_LIMIT_HI    0x0023   /* 32-bit 功率限值 高字          */
#define REG_UNIT_POWER_LIMIT_LO    0x0024   /* 低字                           */
#define REG_UNIT_VOLTAGE_LIMIT     0x0025
#define REG_UNIT_CURRENT_LIMIT     0x0026
#define REG_SYS_BASIC_RESERVED     0x0027   /* 10 regs 预留                   */

/* ----- User / Power Control ----- */
#define REG_OUTPUT_ONOFF           0x0100   /* 0=关, 1=开                     */
#define REG_REGULATION_MODE        0x0101   /* 6=功率 7=电压 8=电流           */
#define REG_SETPOINT_HI            0x0102   /* 32-bit 设定值 高字             */
#define REG_SETPOINT_LO            0x0103
#define REG_CONTROL_MODE           0x0104   /* 2=主机 4=用户端口              */
#define REG_POWER_CTRL_RESERVED    0x0105   /* 10 regs 预留                   */

/* ----- User / 扩展通讯卡 ----- */
#define REG_COMM_WATCHDOG_TIMER    0x0120
#define REG_COMM_CARD_RESERVED     0x0121   /* 10 regs                        */

/* ----- User / User Limit ----- */
#define REG_USER_POWER_LIMIT_HI    0x0140
#define REG_USER_POWER_LIMIT_LO    0x0141
#define REG_USER_VOLTAGE_LIMIT     0x0142
#define REG_USER_CURRENT_LIMIT     0x0143
#define REG_IGNITION_SWITCH        0x0144   /* 0=关, 1=开                     */
#define REG_IGNITION_SETPOINT      0x0145   /* 0=低 1=中 2=高                 */
#define REG_RIPPLE_LEVEL           0x0146   /* 0=高纹波 1=低纹波              */
#define REG_USER_LIMIT_RESERVED    0x0147   /* 10 regs                        */

/* ----- User / Pulse (方波/DC 切换) ----- */
#define REG_PULSE_ENABLE           0x0160
#define REG_DC_POLARITY            0x0161
#define REG_BOOST_VOLT_SETPOINT    0x0162
#define REG_OUTPUT_DEADTIME_1      0x0163
#define REG_OUTPUT_DEADTIME_2      0x0164
#define REG_OUTPUT_FREQUENCY       0x0165
#define REG_PULSE_DUTY_CYCLE       0x0166
#define REG_PULSE_RESERVED         0x0167   /* 10 regs                        */

/* ----- User / Power Pulse (间歇功率输出) ----- */
#define REG_PWR_PULSE_ENABLE       0x0180
#define REG_PWR_PULSE_ON_TIME      0x0181
#define REG_PWR_PULSE_OFF_TIME     0x0182
#define REG_PWR_PULSE_RESERVED     0x0183   /* 10 regs                        */

/* ----- User / Ramp (功率缓启动) ----- */
#define REG_RAMP_ENABLE            0x0190
#define REG_RAMP_STEP              0x0191   /* % of setpoint                  */
#define REG_RAMP_TIME_MS           0x0192   /* ms                             */
#define REG_RAMP_RESERVED          0x0193   /* 10 regs                        */

/* ==================== 寄存器块定义结构 ==================== */
typedef struct {
    uint16_t addr;        /* 起始 Modbus 地址                     */
    uint16_t count;       /* 块内 16-bit 寄存器个数               */
    uint16_t min;         /* 每个寄存器允许的下限                 */
    uint16_t max;         /* 每个寄存器允许的上限                 */
    uint16_t def;         /* 出厂值（整个块所有寄存器都初始化为此值） */
    uint8_t  writable;    /* 0 = 工厂只读, 1 = 用户可写           */
} ModbusRegBlock_t;

/* ==================== 验证结果码 ==================== */
#define MB_VALIDATE_OK             0U      /* 允许写 */
#define MB_VALIDATE_OUT_OF_RANGE   1U      /* 超出 [min,max] → 异常 0x03 */
#define MB_VALIDATE_READONLY       2U      /* 只读区写入   → 异常 0x02 */

/* ==================== API ==================== */
extern const ModbusRegBlock_t Modbus_RegBlocks[];
extern const uint16_t         Modbus_RegBlockCount;

void    Modbus_LoadDefaults(void);
void    Modbus_RefreshFpgaVersionFromFmc(void);
uint8_t Modbus_ValidateRegWrite(uint16_t addr, uint16_t value);

#ifdef __cplusplus
}
#endif

#endif /* __MODBUS_REGS_H__ */
