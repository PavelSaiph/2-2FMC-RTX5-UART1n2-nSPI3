#include "mb_power_ctrl.h"
#include "modbus_regs.h"
#include "FMCREG.h"

/* Modbus 控制权取值：2 表示主机控制，4 表示用户端口控制。 */
#define MB_CTRL_HOST                 2U
#define MB_CTRL_USER                 4U

/* 稳态调节模式取值：功率、电压、电流。 */
#define MB_MODE_POWER                6U
#define MB_MODE_VOLTAGE              7U
#define MB_MODE_CURRENT              8U

/* 无效 FPGA 地址标记：该 Modbus 寄存器不需要同步写入 FPGA。 */
#define MB_REG_INVALID               0xFFFFU

/* 判断一次连续写入范围 [start, start + count) 是否包含指定寄存器地址。 
用于预检验*/
static uint8_t MB_RangeTouches(uint16_t start, uint16_t count, uint16_t addr)
{
    return (addr >= start && addr < (uint16_t)(start + count)) ? 1U : 0U;
}

/* 获取某个地址addr在“本次待写值”中的新值；若本次未写该地址，则取当前Modbus寄存器中的值。 
用于预检验*/
static uint16_t MB_ValueFor(uint16_t start, const uint16_t *values,
                            uint16_t count, uint16_t addr)
{
    if (MB_RangeTouches(start, count, addr) != 0U)
    {
        return values[addr - start];
    }
    return Modbus_RegMap[addr];
}

/* 按 Modbus 高字在前、低字在后的布局组合 32 位无符号数。 */
static uint32_t MB_GetU32(uint16_t hi, uint16_t lo)
{
    return (((uint32_t)hi) << 16) | (uint32_t)lo;
}

/* 判断地址是否属于核心电源控制区：输出开关、稳态模式、设定值、控制模式。 */
static uint8_t MB_IsCoreControlReg(uint16_t addr)
{
    return (addr >= REG_OUTPUT_ONOFF && addr <= REG_CONTROL_MODE) ? 1U : 0U;
}

/* 判断地址是否属于用户限值区：用户功率、电压、电流限值。 
用于预检验*/
static uint8_t MB_IsUserLimitReg(uint16_t addr)
{
    return (addr >= REG_USER_POWER_LIMIT_HI && addr <= REG_USER_CURRENT_LIMIT) ? 1U : 0U;
}

/* 判断地址是否属于点火和纹波参数区。 */
static uint8_t MB_IsIgnitionRippleReg(uint16_t addr)
{
    return (addr >= REG_IGNITION_SWITCH && addr <= REG_RIPPLE_LEVEL) ? 1U : 0U;
}

/* 判断地址是否属于需要转发 FPGA 的扩展控制区：方波、间歇功率、爬坡参数。 */
/* 该地址是否是输出频率的偏移地址 */
static uint8_t MB_IsOutputFrequencyReg(uint16_t addr)
{
    return (addr == REG_OUTPUT_FREQUENCY) ? 1U : 0U;
}

/* 判断地址是否为 32 位设定值的高字或低字。 
用于预检验*/
static uint8_t MB_IsSetpointReg(uint16_t addr)
{
    return (addr == REG_SETPOINT_HI || addr == REG_SETPOINT_LO) ? 1U : 0U;
}

/* 用户端口模式下不额外锁定 output；output 只做 0/1 取值校验后正常提交。 
用于预检验*/
static uint8_t MB_IsProtectedCoreControlReg(uint16_t addr)
{
    (void)addr;
    return 0U;
}

/* 校验稳态调节模式是否为协议允许值。 
用于预检验*/
static uint8_t MB_IsModeValueValid(uint16_t mode)
{
    return (mode == MB_MODE_POWER ||
            mode == MB_MODE_VOLTAGE ||
            mode == MB_MODE_CURRENT) ? 1U : 0U;
}

/* 校验控制权模式是否为协议允许值。 
用于预检验*/
static uint8_t MB_IsControlValueValid(uint16_t control)
{
    return (control == MB_CTRL_HOST || control == MB_CTRL_USER) ? 1U : 0U;
}

/* 计算有效功率限值：用户限值为 0 或超过机器出厂限值时，退回机器出厂限值。 */
static uint32_t MB_GetPowerLimit(const uint16_t *values, uint16_t start,
                                 uint16_t count)
{
    uint32_t unit = MB_GetU32(Modbus_RegMap[REG_UNIT_POWER_LIMIT_HI],
                              Modbus_RegMap[REG_UNIT_POWER_LIMIT_LO]);
    uint32_t user = 0U;

    if (start == REG_USER_POWER_LIMIT_HI && count == 1U)
    {
        user = values[0];
    }
    else
    {
        user = MB_GetU32(MB_ValueFor(start, values, count, REG_USER_POWER_LIMIT_HI),
                         MB_ValueFor(start, values, count, REG_USER_POWER_LIMIT_LO));
    }

    if (unit == 0U)
    {
        unit = UNIT_POWER_LIMIT_W;
    }
    if (user == 0U || user > unit)
    {
        user = unit;
    }
    return user;
}

/* 计算有效电压限值：用户限值为 0 或超过机器出厂限值时，退回机器出厂限值。 */
static uint32_t MB_GetVoltageLimit(const uint16_t *values, uint16_t start,
                                   uint16_t count)
{
    uint32_t unit = Modbus_RegMap[REG_UNIT_VOLTAGE_LIMIT];
    uint32_t user = MB_ValueFor(start, values, count, REG_USER_VOLTAGE_LIMIT);

    if (unit == 0U)
    {
        unit = UNIT_VOLTAGE_LIMIT_V;
    }
    if (user == 0U || user > unit)
    {
        user = unit;
    }
    return user;
}

/* 计算有效电流限值：用户限值为 0 或超过机器出厂限值时，退回机器出厂限值。 */
static uint32_t MB_GetCurrentLimit(const uint16_t *values, uint16_t start,
                                   uint16_t count)
{
    uint32_t unit = Modbus_RegMap[REG_UNIT_CURRENT_LIMIT];
    uint32_t user = MB_ValueFor(start, values, count, REG_USER_CURRENT_LIMIT);

    if (unit == 0U)
    {
        unit = UNIT_CURRENT_LIMIT_01A;
    }
    if (user == 0U || user > unit)
    {
        user = unit;
    }
    return user;
}

/* 按当前或本次待切换的稳态模式校验 32 位设定值。
 * 任何超限写入都会在更新 Modbus_RegMap 或产生 FMC/GPIO 副作用前被拒绝。 */
/* 按调节模式选择对应的用户限值；用户限值本身不受调节模式影响，
 * 只有写 setpoint 时才会根据当前/目标调节模式进入对应分支。 */
/* 主机控制使用机器出厂限值，用户端口使用用户限值。 */
static uint32_t MB_GetUnitLimitByMode(uint16_t mode)
{
    if (mode == MB_MODE_POWER)
    {
        uint32_t unit = MB_GetU32(Modbus_RegMap[REG_UNIT_POWER_LIMIT_HI],
                                  Modbus_RegMap[REG_UNIT_POWER_LIMIT_LO]);
        return (unit != 0U) ? unit : UNIT_POWER_LIMIT_W;
    }
    if (mode == MB_MODE_VOLTAGE)
    {
        uint32_t unit = Modbus_RegMap[REG_UNIT_VOLTAGE_LIMIT];
        return (unit != 0U) ? unit : UNIT_VOLTAGE_LIMIT_V;
    }
    if (mode == MB_MODE_CURRENT)
    {
        uint32_t unit = Modbus_RegMap[REG_UNIT_CURRENT_LIMIT];
        return (unit != 0U) ? unit : UNIT_CURRENT_LIMIT_01A;
    }
    return 0U;
}
/*根据模式获取setpoint的限值*/
static uint8_t MB_GetLimitByMode(uint16_t mode, uint16_t control, uint16_t start,
                                 const uint16_t *values, uint16_t count,
                                 uint32_t *limit)
{
    /*主机模式获取限值*/
    {
        if (control == MB_CTRL_HOST)
        {
            *limit = MB_GetUnitLimitByMode(mode);
            return (*limit != 0U) ? 0U : MB_EX_ILLEGAL_VALUE;
        }
    }
    /*用户端口模式获取限值*/
    {
        if (control != MB_CTRL_USER)
        {
            return MB_EX_ILLEGAL_VALUE;
        }

        if (mode == MB_MODE_POWER)
        {
            *limit = MB_GetPowerLimit(values, start, count);
            return 0U;
        }
        if (mode == MB_MODE_VOLTAGE)
        {
            *limit = MB_GetVoltageLimit(values, start, count);
            return 0U;
        }
        if (mode == MB_MODE_CURRENT)
        {
            *limit = MB_GetCurrentLimit(values, start, count);
            return 0U;
        }
    }

    return MB_EX_ILLEGAL_VALUE;
}

/*确定setpoint设定值是否合法
用于预检验*/
static uint8_t MB_CheckSetpoint(uint16_t start, const uint16_t *values,
                                uint16_t count)
{
    uint16_t mode = MB_ValueFor(start, values, count, REG_REGULATION_MODE);
    uint16_t control = MB_ValueFor(start, values, count, REG_CONTROL_MODE);
    uint32_t setpoint = MB_GetU32(MB_ValueFor(start, values, count, REG_SETPOINT_HI),
                                  MB_ValueFor(start, values, count, REG_SETPOINT_LO));
    uint32_t limit = 0U;
    uint8_t r = MB_GetLimitByMode(mode, control, start, values, count, &limit);

    if (r != 0U)
    {
        return r;
    }

    return (setpoint <= limit) ? 0U : MB_EX_ILLEGAL_VALUE;
}

/* 判断本次写入是否可能影响设定值合法性。
 * 写入设定值、用户限值或稳态模式时，都需要重新按模式检查设定值。 
 用于预检验*/
static uint8_t MB_ShouldCheckSetpointLimit(uint8_t touches_setpoint,
                                           uint16_t next_control)
{
    return (touches_setpoint != 0U && MB_IsControlValueValid(next_control) != 0U) ? 1U : 0U;
}

/*是否使用单寄存器写用户功率限值（一般不会）
用于写入操作*/
static uint8_t MB_IsSingleUserPowerLimitWrite(uint16_t start, uint16_t count)
{
    return (start == REG_USER_POWER_LIMIT_HI && count == 1U) ? 1U : 0U;
}

/* 校验用户功率限值是否超过机器出厂功率限值。
 * 电压/电流用户限值由寄存器表的 min/max 约束；功率是 32 位拆分字段，需要在这里合并检查。 
 用于预检验*/
static uint8_t MB_CheckUserLimitRange(uint16_t start, const uint16_t *values,
                                      uint16_t count)
{
    uint32_t unit_power = MB_GetU32(Modbus_RegMap[REG_UNIT_POWER_LIMIT_HI],
                                   Modbus_RegMap[REG_UNIT_POWER_LIMIT_LO]);
    uint32_t user_power = 0U;

    if (MB_IsSingleUserPowerLimitWrite(start, count) != 0U)
    {
        user_power = values[0];
    }
    else
    {
        user_power = MB_GetU32(MB_ValueFor(start, values, count, REG_USER_POWER_LIMIT_HI),
                               MB_ValueFor(start, values, count, REG_USER_POWER_LIMIT_LO));
    }

    if (unit_power == 0U)
    {
        unit_power = UNIT_POWER_LIMIT_W;
    }

    return (user_power <= unit_power) ? 0U : MB_EX_ILLEGAL_VALUE;
}

/* 校验用户限值、点火和纹波参数的写入权限。
 * 用户限值属于用户端口控制参数，只允许在“用户端口控制 + 输出关闭”时修改。
 * 点火和纹波参数在输出开启期间锁定，避免带载切换。
 * 如果不允许写入，则返回非法（03），如果允许，则返回0
 * 用于预检验 */
static uint8_t MB_CheckUserLimitAccess(uint16_t addr,
                                       uint16_t current_output,
                                       uint16_t current_control,
                                       uint16_t next_output)
{
    if (MB_IsOutputFrequencyReg(addr) != 0U)
    {
        if (current_output != 0U || next_output != 0U)
        {
            return MB_EX_ILLEGAL_VALUE;
        }
    }

    if (MB_IsUserLimitReg(addr) != 0U)
    {
        if (current_output != 0U || current_control != MB_CTRL_USER)
        {
            return MB_EX_ILLEGAL_VALUE;
        }
    }

    if (MB_IsIgnitionRippleReg(addr) != 0U)
    {
        if (current_output != 0U || next_output != 0U)
        {
            return MB_EX_ILLEGAL_VALUE;
        }
    }

    return 0U;
}

/* 校验单个基础控制寄存器的取值范围是否合法：输出开关、稳态模式、控制权模式。
    合法则返回0，否则返回0x03 
    用于预检验*/
static uint8_t MB_CheckBasicValue(uint16_t addr, uint16_t value)
{
    if (addr == REG_OUTPUT_ONOFF && value > 1U)
    {
        return MB_EX_ILLEGAL_VALUE;
    }
    if (addr == REG_REGULATION_MODE && MB_IsModeValueValid(value) == 0U)
    {
        return MB_EX_ILLEGAL_VALUE;
    }
    if (addr == REG_CONTROL_MODE && MB_IsControlValueValid(value) == 0U)
    {
        return MB_EX_ILLEGAL_VALUE;
    }
    return 0U;
}

/* 将 Modbus 寄存器地址映射为 FPGA FMC 控制寄存器地址。
 * 返回 MB_REG_INVALID 表示该地址只更新本地缓存，不写 FPGA。 
 用于写请求*/
static uint16_t MB_FpgaAddrForReg(uint16_t addr)
{
    switch (addr)
    {
    case REG_OUTPUT_ONOFF:        return MB_FPGA_OUTPUT_CTRL;
    case REG_REGULATION_MODE:     return MB_FPGA_REGULATION_MODE;
    case REG_SETPOINT_LO:         return MB_FPGA_SETPOINT_LO;
    case REG_SETPOINT_HI:         return MB_FPGA_SETPOINT_HI;
    case REG_CONTROL_MODE:        return MB_FPGA_CONTROL_MODE;
    case REG_USER_POWER_LIMIT_HI: return FMC_USER_POWER_LIMIT_HI;
    case REG_USER_POWER_LIMIT_LO: return FMC_USER_POWER_LIMIT_LO;
    case REG_USER_VOLTAGE_LIMIT:  return FMC_USER_VOLTAGE_LIMIT;
    case REG_USER_CURRENT_LIMIT:  return FMC_USER_CURRENT_LIMIT;
    case REG_IGNITION_SWITCH:     return MB_FPGA_IGNITION_SWITCH;
    case REG_IGNITION_SETPOINT:   return MB_FPGA_IGNITION_SETPOINT;
    case REG_RIPPLE_LEVEL:        return MB_FPGA_RIPPLE_LEVEL;
    case REG_PULSE_ENABLE:        return MB_FPGA_PULSE_ENABLE;
    case REG_DC_POLARITY:         return MB_FPGA_DC_POLARITY;
    case REG_BOOST_VOLT_SETPOINT: return MB_FPGA_BOOST_VOLT_SETPOINT;
    case REG_OUTPUT_DEADTIME_1:   return MB_FPGA_OUTPUT_DEADTIME_1;
    case REG_OUTPUT_DEADTIME_2:   return MB_FPGA_OUTPUT_DEADTIME_2;
    case REG_OUTPUT_FREQUENCY:    return MB_FPGA_OUTPUT_FREQUENCY;
    case REG_PULSE_DUTY_CYCLE:    return MB_FPGA_PULSE_DUTY_CYCLE;
    case REG_PWR_PULSE_ENABLE:    return MB_FPGA_PWR_PULSE_ENABLE;
    case REG_PWR_PULSE_ON_TIME:   return MB_FPGA_PWR_PULSE_ON_TIME;
    case REG_PWR_PULSE_OFF_TIME:  return MB_FPGA_PWR_PULSE_OFF_TIME;
    case REG_RAMP_ENABLE:         return MB_FPGA_RAMP_ENABLE;
    case REG_RAMP_STEP:           return MB_FPGA_RAMP_STEP;
    case REG_RAMP_TIME_MS:        return MB_FPGA_RAMP_TIME_MS;
    default:                      return MB_REG_INVALID;
    }
}

/* 打开低有效的板级电源/联锁使能链路。 
用于写请求*/
static void MB_EnablePowerPath(void)
{
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET);
}

/* 关闭低有效的板级电源/联锁使能链路。 
用于写请求*/
static void MB_DisablePowerPath(void)
{
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET);
}

/* 写入前业务校验入口。
 * 检验本次写入是否涉及到调节模式、控制模式以及用户限值*
 * 如果涉及到，返回1（后续需要进一步清除setpoint），否则返回0
 * 用于写请求*/
static uint8_t MB_ShouldClearSetpointAfterWrite(uint16_t start, uint16_t count)
{
    if (MB_RangeTouches(start, count, REG_REGULATION_MODE) != 0U ||
        MB_RangeTouches(start, count, REG_CONTROL_MODE) != 0U)
    {
        return 1U;
    }

    return (start <= REG_USER_CURRENT_LIMIT &&
            (uint16_t)(start + count) > REG_USER_POWER_LIMIT_HI) ? 1U : 0U;
}

/*清零setpoint并同步给FPGA
用于写请求*/
static void MB_ClearSetpoint(void)
{
    Modbus_RegMap[REG_SETPOINT_HI] = 0U;
    Modbus_RegMap[REG_SETPOINT_LO] = 0U;
    FMC_WRITE(MB_FPGA_SETPOINT_HI, 0U);
    FMC_WRITE(MB_FPGA_SETPOINT_LO, 0U);
}

/* 数据写入的预检验
 * 检验的条目如下：
 * 1.判断output/调节模式/控制模式的取值是否合法
 * 2.是否允许输出频率(0x165)或用户限值的写入
 * 3.输出开启期间禁止切换控制模式
 * 4.输出开启期间禁止切换调节模式
 * 5.校验设定值是否越界
 * 6.校验用户功率限值是否超过机器出厂功率限值*/
uint8_t MB_Power_PreWrite(uint16_t start, const uint16_t *values, uint16_t count)
{
    uint8_t touches_control = 0U;
    uint8_t touches_setpoint = 0U;
    uint8_t touches_user_limit = 0U;
    uint16_t current_output = Modbus_RegMap[REG_OUTPUT_ONOFF];
    uint16_t current_control = Modbus_RegMap[REG_CONTROL_MODE];
    uint16_t next_output = MB_ValueFor(start, values, count, REG_OUTPUT_ONOFF);
    uint16_t next_mode = MB_ValueFor(start, values, count, REG_REGULATION_MODE);
    uint16_t next_control = MB_ValueFor(start, values, count, REG_CONTROL_MODE);

    /* 逐寄存器检查基础取值，并记录本次写入触及了哪些业务模块。 */
    for (uint16_t i = 0U; i < count; i++)
    {
        uint16_t addr = start + i;
        uint16_t val = values[i];
        uint8_t r = MB_CheckBasicValue(addr, val);/*判断output/调节模式/控制模式的取值是否合法*/

        if (r != 0U)
        {
            return r;
        }
        /*暂时没有用到*/
        if (MB_IsProtectedCoreControlReg(addr) != 0U)
        {
            touches_control = 1U;
        }
        /*是否修改setpoint*/
        if (MB_IsSetpointReg(addr) != 0U)
        {
            touches_setpoint = 1U;
        }
        /*是否修改用户限值*/
        if (MB_IsUserLimitReg(addr) != 0U)
        {
            touches_user_limit = 1U;
        }
        
        /*是否允许输出频率(0x165)或用户限值的写入*/
        r = MB_CheckUserLimitAccess(addr, current_output, current_control, next_output);
        if (r != 0U)
        {
            return r;
        }
    }

    /* 用户端口控制时，除“切回主机控制”外，不允许再由 Modbus 改动控制参数。 
    目前关闭*/
    if (touches_control != 0U && current_control == MB_CTRL_USER)
    {
        uint8_t only_switch_to_host =
            (count == 1U && start == REG_CONTROL_MODE && values[0] == MB_CTRL_HOST) ? 1U : 0U;

        if (only_switch_to_host == 0U)
        {
            return MB_EX_ILLEGAL_VALUE;
        }
    }

    /* 输出开启期间禁止切换控制模式，避免功率链路运行中改变控制来源。 */
    if (MB_RangeTouches(start, count, REG_CONTROL_MODE) != 0U &&
        (current_output != 0U || next_output != 0U))
    {
        return MB_EX_ILLEGAL_VALUE;
    }

    /* 输出开启期间禁止切换调节模式，避免带载从功率/电压/电流模式互切。 */
    if (MB_RangeTouches(start, count, REG_REGULATION_MODE) != 0U &&
        (current_output != 0U || next_output != 0U))
    {
        return MB_EX_ILLEGAL_VALUE;
    }

    /* 用整包写入后的目标状态再做一次核心取值兜底校验。 */
    if (next_output > 1U ||
        MB_IsModeValueValid(next_mode) == 0U ||
        MB_IsControlValueValid(next_control) == 0U)
    {
        return MB_EX_ILLEGAL_VALUE;
    }

    /* 设定值、限值或稳态模式变化时，按最终模式重新校验设定值是否越界。 */
    if (MB_ShouldCheckSetpointLimit(touches_setpoint, next_control) != 0U)
    {
        uint8_t r = MB_CheckSetpoint(start, values, count);
        if (r != 0U)
        {
            return r;
        }
    }

    /* 用户功率限值是 32 位字段，整包写完后的组合值不能超过机器出厂限值。 */
    if (touches_user_limit != 0U)
    {
        uint8_t r = MB_CheckUserLimitRange(start, values, count);
        if (r != 0U)
        {
            return r;
        }
    }

    return 0U;
}

/* 提交已经通过校验的写请求并响应相应操作。
 * 提交流程：更新 Modbus 本地缓存，执行电源 GPIO 副作用，再把可映射寄存器写入 FPGA。 */
void MB_Power_CommitWrite(uint16_t start, const uint16_t *values, uint16_t count)
{
    uint8_t clear_setpoint = MB_ShouldClearSetpointAfterWrite(start, count);

    /*写用户功率限值*/
    if (MB_IsSingleUserPowerLimitWrite(start, count) != 0U)
    {
        Modbus_RegMap[REG_USER_POWER_LIMIT_HI] = 0U;
        Modbus_RegMap[REG_USER_POWER_LIMIT_LO] = values[0];
        FMC_WRITE(FMC_USER_POWER_LIMIT_HI, 0U);
        FMC_WRITE(FMC_USER_POWER_LIMIT_LO, values[0]);
        if (clear_setpoint != 0U)
        {
            MB_ClearSetpoint();
        }
        return;
    }

    for (uint16_t i = 0U; i < count; i++)
    {
        uint16_t addr = start + i;
        uint16_t val = values[i];
        uint16_t fpga_addr = MB_FpgaAddrForReg(addr);

        /*修改寄存器中相应的值*/
        Modbus_RegMap[addr] = val;

        /*这一部分是USART2-HMI写完之后ARM需要执行相应操作begin*/
        /* 输出开关直接控制板级低有效电源/联锁使能链路。 */
        if (addr == REG_OUTPUT_ONOFF && val == 1U)
        {
            MB_EnablePowerPath();
        }
        else if (addr == REG_OUTPUT_ONOFF && val == 0U)
        {
            MB_DisablePowerPath();
        }

        /*这一部分是USART2-HMI写完之后ARM需要执行的相应操作end*/

        /* 只有存在映射关系的业务寄存器才同步写入 FPGA。 */
        if (fpga_addr != MB_REG_INVALID)
        {
            FMC_WRITE(fpga_addr, val);
        }
    }

    if (clear_setpoint != 0U)
    {
        MB_ClearSetpoint();
    }
}

/* 读响应前刷新入口。
 * 当前电源控制寄存器以 ARM 侧已提交缓存为准，暂不从 FPGA 状态区回读覆盖。
 * 保留 start/count 参数，便于后续 FPGA 增加独立状态回显后按地址范围刷新。 */
void MB_Power_RefreshBeforeRead(uint16_t start, uint16_t count)
{
    (void)start;
    (void)count;
}
