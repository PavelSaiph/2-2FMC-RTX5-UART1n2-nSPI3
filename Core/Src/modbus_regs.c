/**
  ******************************************************************************
  * @file    modbus_regs.c
  * @brief   自主中频电源控制板与通讯卡交互数据表
  *          来源文档：自主中频电源控制板与通讯卡交互数据列表 V1.0
  *
  * 表项格式：{ 起始地址, 占用寄存器数, 下限, 上限, 出厂值, 可写 }
  *   可写 = 0 : 工厂只读（写入返回异常码 0x02 非法地址）
  *   可写 = 1 : 用户可写（超出范围返回异常码 0x03 非法数据）
  ******************************************************************************
  */

#include "modbus_regs.h"
#include "FMCREG.h"

/* 字符串字段按每个 Modbus 寄存器存 2 个 ASCII 字符：高字节在前、低字节在后。 */
static void Modbus_SetAsciiRegs(uint16_t addr, uint16_t count, const char *text)
{
    for (uint16_t i = 0U; i < count; i++)
    {
        uint8_t hi = (uint8_t)' ';
        uint8_t lo = (uint8_t)' ';
        uint16_t text_index = (uint16_t)(i * 2U);

        if (text[text_index] != '\0')
        {
            hi = (uint8_t)text[text_index];
            if (text[text_index + 1U] != '\0')
            {
                lo = (uint8_t)text[text_index + 1U];
            }
        }

        if ((uint32_t)addr + i < MAX_REG_ADDR)
        {
            Modbus_RegMap[addr + i] = ((uint16_t)hi << 8) | lo;
        }
    }
}

/* 设备信息页的只读字段默认值；FPGA 版本后续可改为从 FPGA 版本 FMC 地址读取。 */
static void Modbus_LoadDeviceInfoDefaults(void)
{
    Modbus_SetAsciiRegs(REG_UNIT_TYPE,     10U, "IMF-15K-600V");
    Modbus_SetAsciiRegs(REG_UNIT_NUMBER,    5U, "SN00000001");
    Modbus_SetAsciiRegs(REG_FPGA1_VERSION,  5U, "FPGA1 1.0");
    Modbus_SetAsciiRegs(REG_FPGA2_VERSION,  5U, "FPGA2 1.0");
    Modbus_SetAsciiRegs(REG_ARM1_VERSION,   5U, "ARM1  1.0");
    Modbus_SetAsciiRegs(REG_ARM2_VERSION,   5U, "ARM2  1.0");
}

static uint8_t Modbus_IsPrintableAsciiWord(uint16_t word)
{
    uint8_t hi = (uint8_t)((word >> 8) & 0xFFU);
    uint8_t lo = (uint8_t)(word & 0xFFU);

    return (hi >= 0x20U && hi <= 0x7EU && lo >= 0x20U && lo <= 0x7EU) ? 1U : 0U;
}

static uint8_t Modbus_IsValidFpgaVersionWords(const uint16_t *words)
{
    for (uint16_t i = 0U; i < 5U; i++)
    {
        if (words[i] == 0x0000U || words[i] == 0xFFFFU ||
            Modbus_IsPrintableAsciiWord(words[i]) == 0U)
        {
            return 0U;
        }
    }

    return ((uint8_t)((words[0] >> 8) & 0xFFU) == (uint8_t)'F' &&
            (uint8_t)(words[0] & 0xFFU) == (uint8_t)'P' &&
            (uint8_t)((words[1] >> 8) & 0xFFU) == (uint8_t)'G' &&
            (uint8_t)(words[1] & 0xFFU) == (uint8_t)'A') ? 1U : 0U;
}

/* 从 FPGA 版本 FMC 地址读取 5 个版本字；只有有效 ASCII 版本号才覆盖默认值。 */
static void Modbus_LoadFpgaVersionWords(uint16_t modbus_addr, uint16_t fpga_addr0)
{
    uint16_t words[5];

    for (uint16_t i = 0U; i < 5U; i++)
    {
        words[i] = FMC_READ((uint16_t)(fpga_addr0 + i));
    }

    if (Modbus_IsValidFpgaVersionWords(words) == 0U)
    {
        return;
    }

    for (uint16_t i = 0U; i < 5U; i++)
    {
        if ((uint32_t)modbus_addr + i < MAX_REG_ADDR)
        {
            Modbus_RegMap[modbus_addr + i] = words[i];
        }
    }
}


/* FPGA 握手完成后调用，用 FPGA 实际版本号覆盖默认版本号。 */
void Modbus_RefreshFpgaVersionFromFmc(void)
{
    Modbus_LoadFpgaVersionWords(REG_FPGA1_VERSION, FPGA1_VERSION_REG0);
    Modbus_LoadFpgaVersionWords(REG_FPGA2_VERSION, FPGA2_VERSION_REG0);
}

/* ============================================================================
 *   寄存器块定义表
 *   说明中字节数按原始交互表，多字节字段会拆成多个 16-bit 寄存器
 *   32-bit 整数：高字在前，低字在后
 *   字符串：每个寄存器放 2 字符（高字节 = 前一个字符，低字节 = 后一个字符）
 * ============================================================================ */
const ModbusRegBlock_t Modbus_RegBlocks[] =
{
    /* ============================================================================
     * System / Basic - 设备基础信息（客户不能修改，出厂配置）
     * ============================================================================ */
    /*  地址                         个数  min     max     def     writable */
    /*  -----                        ----  -----   -----   -----   -------- */
    { REG_UNIT_TYPE,                  10,  0,      0xFFFF, 0,      0 }, /* Unit Type 电源型号 (20B ASCII) */
    { REG_UNIT_NUMBER,                 5,  0,      0xFFFF, 0,      0 }, /* Unit Number 序列号 (10B)       */
    { REG_FPGA1_VERSION,               5,  0,      0xFFFF, 0,      0 }, /* FPGA1 版本号                   */
    { REG_FPGA2_VERSION,               5,  0,      0xFFFF, 0,      0 }, /* FPGA2 版本号                   */
    { REG_ARM1_VERSION,                5,  0,      0xFFFF, 0,      0 }, /* ARM1 版本号                    */
    { REG_ARM2_VERSION,                5,  0,      0xFFFF, 0,      0 }, /* ARM2 版本号                    */

    /* 机器功率限值（4B，32-bit W）- 出厂数值 15000 W */
    { REG_UNIT_POWER_LIMIT_HI,         1,  0,      0xFFFF, HI16(UNIT_POWER_LIMIT_W), 0 },
    { REG_UNIT_POWER_LIMIT_LO,         1,  0,      0xFFFF, LO16(UNIT_POWER_LIMIT_W), 0 },

    /* 机器电压限值（2B，V） */
    { REG_UNIT_VOLTAGE_LIMIT,          1,  0,      0xFFFF, UNIT_VOLTAGE_LIMIT_V,     0 },

    /* 机器电流限值（2B，0.1A） */
    { REG_UNIT_CURRENT_LIMIT,          1,  0,      0xFFFF, UNIT_CURRENT_LIMIT_01A,   0 },

    /* 预留 20B */
    { REG_SYS_BASIC_RESERVED,         10,  0,      0xFFFF, 0,      0 },

    /* ============================================================================
     * User / Power Control - 电源输出控制
     * ============================================================================ */
    { REG_OUTPUT_ONOFF,                1,  0,      1,      0,      1 }, /* 0=关功率, 1=开功率          */
    { REG_REGULATION_MODE,             1,  6,      8,      6,      1 }, /* 6=功率 7=电压 8=电流        */
    { REG_SETPOINT_HI,                 1,  0,      0xFFFF, 0,      1 }, /* 设定值高字 (32-bit)          */
    { REG_SETPOINT_LO,                 1,  0,      0xFFFF, 0,      1 }, /* 设定值低字                   */
    { REG_CONTROL_MODE,                1,  2,      4,      2,      1 }, /* 2=主机控制 4=用户端口控制    */
    { REG_POWER_CTRL_RESERVED,        10,  0,      0xFFFF, 0,      1 }, /* 预留 20B                     */

    /* ============================================================================
     * User / 扩展通讯卡
     * ============================================================================ */
    { REG_COMM_WATCHDOG_TIMER,         1,  0,      0xFFFF, 0,      1 }, /* 通讯看门狗超时，0=禁用       */
    { REG_COMM_CARD_RESERVED,         10,  0,      0xFFFF, 0,      1 }, /* 预留 20B                     */

    /* ============================================================================
     * User / User Limit - 用户限值（不得超过机器出厂限值）
     * ============================================================================ */
    { REG_USER_POWER_LIMIT_HI,         1,  0,      0xFFFF, HI16(UNIT_POWER_LIMIT_W), 1 },
    { REG_USER_POWER_LIMIT_LO,         1,  0,      0xFFFF, LO16(UNIT_POWER_LIMIT_W), 1 },
    { REG_USER_VOLTAGE_LIMIT,          1,  0,      UNIT_VOLTAGE_LIMIT_V,   UNIT_VOLTAGE_LIMIT_V,   1 },
    { REG_USER_CURRENT_LIMIT,          1,  0,      UNIT_CURRENT_LIMIT_01A, UNIT_CURRENT_LIMIT_01A, 1 },
    { REG_IGNITION_SWITCH,             1,  0,      1,      0,      1 }, /* 0=关 1=开                    */
    { REG_IGNITION_SETPOINT,           1,  0,      2,      0,      1 }, /* 0=低 1=中 2=高               */
    { REG_RIPPLE_LEVEL,                1,  0,      1,      0,      1 }, /* 0=高纹波 1=低纹波            */
    { REG_USER_LIMIT_RESERVED,        10,  0,      0xFFFF, 0,      1 }, /* 预留 20B                     */

    /* ============================================================================
     * User / Pulse - 方波/DC 切换
     * ============================================================================ */
    { REG_PULSE_ENABLE,                1,  0,      1,      0,      1 }, /* 0=DC 1=方波                  */
    { REG_DC_POLARITY,                 1,  0,      1,      0,      1 }, /* 直流极性                     */
    { REG_BOOST_VOLT_SETPOINT,         1,  0,      0xFFFF, 0,      1 },
    { REG_OUTPUT_DEADTIME_1,           1,  0,      0xFFFF, 0,      1 },
    { REG_OUTPUT_DEADTIME_2,           1,  0,      0xFFFF, 0,      1 },
    { REG_OUTPUT_FREQUENCY,            1,  0,      0xFFFF, 0,      1 },
    { REG_PULSE_DUTY_CYCLE,            1,  0,      100,    50,     1 }, /* 占空比 0-100 %               */
    { REG_PULSE_RESERVED,             10,  0,      0xFFFF, 0,      1 }, /* 预留 20B                     */

    /* ============================================================================
     * User / Power Pulse - 间歇性功率输出
     * ============================================================================ */
    { REG_PWR_PULSE_ENABLE,            1,  0,      1,      0,      1 },
    { REG_PWR_PULSE_ON_TIME,           1,  0,      0xFFFF, 0,      1 }, /* 功率脉冲打开时长 (2B)        */
    { REG_PWR_PULSE_OFF_TIME,          1,  0,      0xFFFF, 0,      1 }, /* 功率脉冲关闭时长 (2B)        */
    { REG_PWR_PULSE_RESERVED,         10,  0,      0xFFFF, 0,      1 }, /* 预留 20B                     */

    /* ============================================================================
     * User / Ramp - 功率缓启动（爬坡）
     * ============================================================================ */
    { REG_RAMP_ENABLE,                 1,  0,      1,      0,      1 },
    { REG_RAMP_STEP,                   1,  1,      100,    10,     1 }, /* 每步占设定值的 % (1-100)     */
    { REG_RAMP_TIME_MS,                1,  0,      0xFFFF, 1000,   1 }, /* 总爬坡时长 ms                */
    { REG_RAMP_RESERVED,              10,  0,      0xFFFF, 0,      1 }, /* 预留 20B                     */
};

const uint16_t Modbus_RegBlockCount =
    sizeof(Modbus_RegBlocks) / sizeof(Modbus_RegBlocks[0]);

/* ============================================================================
 *                             API 实现
 * ============================================================================ */

/* 上电调用一次：按表项把 Modbus_RegMap 批量填成出厂值 */
void Modbus_LoadDefaults(void)
{
    for (uint16_t b = 0; b < Modbus_RegBlockCount; b++)
    {
        const ModbusRegBlock_t *blk = &Modbus_RegBlocks[b];
        for (uint16_t i = 0; i < blk->count; i++)
        {
            uint32_t a = (uint32_t)blk->addr + i;
            if (a < MAX_REG_ADDR)
            {
                Modbus_RegMap[a] = blk->def;
            }
        }
    }

    Modbus_LoadDeviceInfoDefaults();
}

/* 校验一次写入
 * 返回 0 = 允许, 1 = 超范围 (0x03), 2 = 只读 (0x02)
 * 地址未落入任何定义块 -> 视为未管辖，允许写入
 */
uint8_t Modbus_ValidateRegWrite(uint16_t addr, uint16_t value)
{
    for (uint16_t b = 0; b < Modbus_RegBlockCount; b++)
    {
        const ModbusRegBlock_t *blk = &Modbus_RegBlocks[b];
        if (addr >= blk->addr && addr < (uint16_t)(blk->addr + blk->count))
        {
            if (blk->writable == 0U)
            {
                return MB_VALIDATE_READONLY;
            }
            if (value < blk->min || value > blk->max)
            {
                return MB_VALIDATE_OUT_OF_RANGE;
            }
            return MB_VALIDATE_OK;
        }
    }
    return MB_VALIDATE_OK;
}
