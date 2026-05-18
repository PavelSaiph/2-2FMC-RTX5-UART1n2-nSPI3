#ifndef __MB_POWER_CTRL_H__
#define __MB_POWER_CTRL_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "modbus_slave.h"

/* 数据写入的预检验
 * 检验的条目如下：
 * 1.判断output/调节模式/控制模式的取值是否合法
 * 2.是否允许输出频率(0x165)或用户限值的写入
 * 3.输出开启期间禁止切换控制模式
 * 4.输出开启期间禁止切换调节模式
 * 5.校验设定值是否越界
 * 6.校验用户功率限值是否超过机器出厂功率限值*/
uint8_t MB_Power_PreWrite(uint16_t start, const uint16_t *values, uint16_t count);

/* 将已校验通过的写请求提交到 Modbus 缓存、GPIO 输出和 FPGA FMC 命令。 */
void MB_Power_CommitWrite(uint16_t start, const uint16_t *values, uint16_t count);

/* 在构造读响应前刷新业务控制状态。 */
void MB_Power_RefreshBeforeRead(uint16_t start, uint16_t count);

#ifdef __cplusplus
}
#endif

#endif /* __MB_POWER_CTRL_H__ */
