/**
  ******************************************************************************
  * @file    modbus_slave.c
  * @brief   Modbus RTU slave on USART2 (RS485) - Armfly style port
  ******************************************************************************
  */

#include "modbus_slave.h"
#include "modbus_regs.h"
#include "usart.h"
#include "app_tasks.h"
#include <string.h>
#include "FMCREG.h"
#include "mb_power_ctrl.h"

extern uint16_t *p_FMC_Latched;

/* ================= 内部变量 ================= */
TIM_HandleTypeDef htim6;

static uint8_t  mb_rx_buf[MODBUS_RX_BUF_SIZE];
static volatile uint16_t mb_rx_len       = 0;
static volatile uint16_t mb_silence_cnt  = 0;
static volatile uint8_t  mb_frame_ready  = 0;
static volatile uint8_t  mb_rx_overrun   = 0;   /* RX 缓冲溢出标志（帧超过 256B） */

/* 兜底看门狗：nRTS 连续 HIGH 计时（ms），超过阈值强制拉低 */
#define MB_NRTS_MAX_HIGH_MS     500
static volatile uint16_t mb_nrts_high_ms = 0;

static uint8_t  mb_tx_buf[MODBUS_TX_BUF_SIZE];

/* ================= CRC16 Modbus (多项式 0xA001) ================= */
static uint16_t Modbus_CRC16(const uint8_t *data, uint16_t len)
{
    uint16_t crc = 0xFFFF;
    uint16_t i;
    uint8_t  j;
    for (i = 0; i < len; i++)
    {
        crc ^= data[i];
        for (j = 0; j < 8; j++)
        {
            if (crc & 0x0001) crc = (crc >> 1) ^ 0xA001;
            else              crc >>= 1;
        }
    }
    return crc;
}

/* ================= TIM6 初始化（1ms tick） =================
 * APB1 Timer 时钟 = 200MHz（APB1=100MHz，APB1 prescaler>1 时 ×2）
 * Prescaler = 200-1 → 1MHz tick
 * Period    = 1000-1 → 1ms 中断一次
 */
static void MB_TIM6_Init(void)
{
    __HAL_RCC_TIM6_CLK_ENABLE();

    htim6.Instance               = TIM6;
    htim6.Init.Prescaler         = 200 - 1;
    htim6.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim6.Init.Period            = 1000 - 1;
    htim6.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
    {
        Error_Handler();
    }

    HAL_NVIC_SetPriority(TIM6_DAC_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);

    HAL_TIM_Base_Start_IT(&htim6);
}

/* ================= 公共 API ================= */

void Modbus_Init(void)
{
    Modbus_ResetRxState();
    MB_TIM6_Init();
    MyUsart_EnableReceiveByte(&huart2);
}

/* 外部调用（UART 错误回调）：丢弃当前半成品帧 */
void Modbus_ResetRxState(void)
{
    mb_rx_len       = 0;
    mb_silence_cnt  = 0;
    mb_frame_ready  = 0;
    mb_rx_overrun   = 0;
}

/* UART RX 中断里调用；每来一个字节塞进 buffer 并复位空闲计数 */
void Modbus_RxByte(uint8_t byte)
{
    /* 上一帧还没处理，丢弃新字节以免覆盖 */
    if (mb_frame_ready) return;

    if (mb_rx_len < MODBUS_RX_BUF_SIZE)
    {
        mb_rx_buf[mb_rx_len++] = byte;
    }
    else
    {
        /* 缓冲溢出：做记号，让下次空闲超时直接整帧丢弃 */
        mb_rx_overrun = 1;
    }
    mb_silence_cnt = 0;
}

/* TIM6 中断里调用，每 1ms 一次 */
void Modbus_TimerTick_1ms(void)
{
    /* --- 帧空闲计时 --- */
    if (mb_rx_len > 0 && !mb_frame_ready)
    {
        mb_silence_cnt++;
        if (mb_silence_cnt >= MODBUS_FRAME_SILENCE_MS)
        {
            mb_frame_ready = 1;
            /* 通知后台任务：新帧就绪 */
            if (sem_mb_frame != NULL)
            {
                (void)osSemaphoreRelease(sem_mb_frame);
            }
        }
    }

    /* --- 兜底看门狗：nRTS 卡 HIGH 超过 500ms 强制拉低，释放总线 --- */
    if (HAL_GPIO_ReadPin(USART2_nRTS_GPIO_Port, USART2_nRTS_Pin) == GPIO_PIN_SET)
    {
        mb_nrts_high_ms++;
        if (mb_nrts_high_ms > MB_NRTS_MAX_HIGH_MS)
        {
            HAL_GPIO_WritePin(USART2_nRTS_GPIO_Port, USART2_nRTS_Pin, GPIO_PIN_RESET);
            mb_nrts_high_ms = 0;
        }
    }
    else
    {
        mb_nrts_high_ms = 0;
    }
}

/* ================= 发送帮助函数 ================= */

static void Modbus_SendWithCRC(uint16_t len_before_crc)
{
    uint16_t crc = Modbus_CRC16(mb_tx_buf, len_before_crc);
    mb_tx_buf[len_before_crc]     = (uint8_t)(crc & 0xFF);       /* Modbus CRC 低字节在前 */
    mb_tx_buf[len_before_crc + 1] = (uint8_t)((crc >> 8) & 0xFF);
    MyUsart_SendData(&huart2, mb_tx_buf, len_before_crc + 2);
}

/*返回错误数据帧*/
static void Modbus_SendException(uint8_t addr, uint8_t fc, uint8_t ex_code)
{
    mb_tx_buf[0] = addr;
    mb_tx_buf[1] = fc | 0x80;
    mb_tx_buf[2] = ex_code;
    Modbus_SendWithCRC(3);
}

/* ================= 线圈 读/写 辅助函数 ================= */
static uint8_t MB_ReadCoil(uint16_t addr)
{
    return (Modbus_CoilMap[addr >> 3] >> (addr & 0x07)) & 0x01;
}

static void MB_WriteCoil(uint16_t addr, uint8_t val)
{
    if (val) Modbus_CoilMap[addr >> 3] |=  (uint8_t)(1U << (addr & 0x07));
    else     Modbus_CoilMap[addr >> 3] &= (uint8_t)~(1U << (addr & 0x07));
}

/* ================= 功能码 0x01 / 0x02：读线圈 / 读离散输入 =================
 * 两者协议格式完全相同，只是响应里 fc 不一样。
 * 本从机两者共用同一张位图 Modbus_CoilMap。
 */
static void Handle_ReadBitArea(uint8_t addr, uint8_t fc,
                                const uint8_t *frame, uint16_t len)
{
    if (len != 8)
    {
        Modbus_SendException(addr, fc, MB_EX_ILLEGAL_VALUE);
        return;
    }

    uint16_t start = ((uint16_t)frame[2] << 8) | frame[3];
    uint16_t count = ((uint16_t)frame[4] << 8) | frame[5];

    if (count == 0 || count > 2000)
    {
        Modbus_SendException(addr, fc, MB_EX_ILLEGAL_VALUE);
        return;
    }
    if ((uint32_t)start + count > MAX_COIL_ADDR)
    {
        Modbus_SendException(addr, fc, MB_EX_ILLEGAL_ADDRESS);
        return;
    }

    uint16_t byte_count = (count + 7) / 8;
    mb_tx_buf[0] = addr;
    mb_tx_buf[1] = fc;
    mb_tx_buf[2] = (uint8_t)byte_count;

    /* 清零 + 按位填充 */
    for (uint16_t i = 0; i < byte_count; i++) mb_tx_buf[3 + i] = 0;
    for (uint16_t i = 0; i < count; i++)
    {
        if (MB_ReadCoil(start + i))
        {
            mb_tx_buf[3 + (i >> 3)] |= (uint8_t)(1U << (i & 0x07));
        }
    }
    Modbus_SendWithCRC(3 + byte_count);
}

/* ================= 功能码 0x04：读输入寄存器 =================
 * 格式与 0x03 相同，本从机里与保持寄存器共享 Modbus_RegMap。
 */
static void Handle_ReadInputReg(uint8_t addr, const uint8_t *frame, uint16_t len)
{
    if (len != 8)
    {
        Modbus_SendException(addr, MB_FC_READ_INPUT_REG, MB_EX_ILLEGAL_VALUE);
        return;
    }

    uint16_t start = ((uint16_t)frame[2] << 8) | frame[3];
    uint16_t count = ((uint16_t)frame[4] << 8) | frame[5];

    if (count == 0 || count > 125)
    {
        Modbus_SendException(addr, MB_FC_READ_INPUT_REG, MB_EX_ILLEGAL_VALUE);
        return;
    }
    if ((uint32_t)start + count > MAX_REG_ADDR)
    {
        Modbus_SendException(addr, MB_FC_READ_INPUT_REG, MB_EX_ILLEGAL_ADDRESS);
        return;
    }

    /* 组包读取数据前，先从 FPGA 刷新业务控制寄存器。 */
    MB_Power_RefreshBeforeRead(start, count);

    mb_tx_buf[0] = addr;
    mb_tx_buf[1] = MB_FC_READ_INPUT_REG;
    mb_tx_buf[2] = (uint8_t)(count * 2);
    for (uint16_t i = 0; i < count; i++)
    {
        uint16_t v = Modbus_RegMap[start + i];
        mb_tx_buf[3 + i * 2] = (uint8_t)((v >> 8) & 0xFF);
        mb_tx_buf[4 + i * 2] = (uint8_t)(v & 0xFF);
    }
    Modbus_SendWithCRC(3 + count * 2);
}

/* ================= 功能码 0x05：写单个线圈 =================
 * 请求：addr 05 | coilH coilL | valH valL | crcL crcH
 * 线圈值规定只能是 0xFF00（ON）或 0x0000（OFF），其他一律非法
 */
static void Handle_WriteSingleCoil(uint8_t addr, const uint8_t *frame, uint16_t len)
{
    if (len != 8)
    {
        Modbus_SendException(addr, MB_FC_WRITE_SINGLE_COIL, MB_EX_ILLEGAL_VALUE);
        return;
    }

    uint16_t coil = ((uint16_t)frame[2] << 8) | frame[3];
    uint16_t val  = ((uint16_t)frame[4] << 8) | frame[5];

    if (val != 0xFF00 && val != 0x0000)
    {
        Modbus_SendException(addr, MB_FC_WRITE_SINGLE_COIL, MB_EX_ILLEGAL_VALUE);
        return;
    }
    if (coil >= MAX_COIL_ADDR)
    {
        Modbus_SendException(addr, MB_FC_WRITE_SINGLE_COIL, MB_EX_ILLEGAL_ADDRESS);
        return;
    }

    MB_WriteCoil(coil, (val == 0xFF00) ? 1U : 0U);

    /* 响应：与请求前 6 字节相同 */
    memcpy(mb_tx_buf, frame, 6);
    Modbus_SendWithCRC(6);
}

/* ================= 功能码 0x0F：写多个线圈 =================
 * 请求：addr 0F | startH startL | cntH cntL | bytes | [data...] | crcL crcH
 */
static void Handle_WriteMultiCoils(uint8_t addr, const uint8_t *frame, uint16_t len)
{
    if (len < 9)
    {
        Modbus_SendException(addr, MB_FC_WRITE_MULTI_COILS, MB_EX_ILLEGAL_VALUE);
        return;
    }

    uint16_t start = ((uint16_t)frame[2] << 8) | frame[3];
    uint16_t count = ((uint16_t)frame[4] << 8) | frame[5];
    uint8_t  bytes = frame[6];

    uint16_t expect_bytes = (count + 7) / 8;
    if (count == 0 || count > 1968 || bytes != expect_bytes)
    {
        Modbus_SendException(addr, MB_FC_WRITE_MULTI_COILS, MB_EX_ILLEGAL_VALUE);
        return;
    }
    if (len != (uint16_t)(7 + bytes + 2))
    {
        Modbus_SendException(addr, MB_FC_WRITE_MULTI_COILS, MB_EX_ILLEGAL_VALUE);
        return;
    }
    if ((uint32_t)start + count > MAX_COIL_ADDR)
    {
        Modbus_SendException(addr, MB_FC_WRITE_MULTI_COILS, MB_EX_ILLEGAL_ADDRESS);
        return;
    }

    for (uint16_t i = 0; i < count; i++)
    {
        uint8_t bit = (frame[7 + (i >> 3)] >> (i & 0x07)) & 0x01;
        MB_WriteCoil(start + i, bit);
    }

    mb_tx_buf[0] = addr;
    mb_tx_buf[1] = MB_FC_WRITE_MULTI_COILS;
    mb_tx_buf[2] = (uint8_t)((start >> 8) & 0xFF);
    mb_tx_buf[3] = (uint8_t)(start & 0xFF);
    mb_tx_buf[4] = (uint8_t)((count >> 8) & 0xFF);
    mb_tx_buf[5] = (uint8_t)(count & 0xFF);
    Modbus_SendWithCRC(6);
}

/* ================= 功能码 0x03：读保持寄存器 ================= */
static void Handle_ReadHoldingReg(uint8_t addr, const uint8_t *frame, uint16_t len)
{
    /* 请求格式：addr fc | regH regL | cntH cntL | crcL crcH   （共 8 字节） */
    if (len != 8)
    {
        Modbus_SendException(addr, MB_FC_READ_HOLDING_REG, MB_EX_ILLEGAL_VALUE);
        return;
    }

    uint16_t start = ((uint16_t)frame[2] << 8) | frame[3];
    uint16_t count = ((uint16_t)frame[4] << 8) | frame[5];

    if (count == 0 || count > 125)
    {
        Modbus_SendException(addr, MB_FC_READ_HOLDING_REG, MB_EX_ILLEGAL_VALUE);
        return;
    }
    if ((uint32_t)start + count > MAX_REG_ADDR)
    {
        Modbus_SendException(addr, MB_FC_READ_HOLDING_REG, MB_EX_ILLEGAL_ADDRESS);
        return;
    }

    /* 读取 0x0100-0x0104 前，同步 FPGA 侧最新控制状态。 */
    MB_Power_RefreshBeforeRead(start, count);

    /* 响应：addr fc byte_count [data...] crcL crcH */
    mb_tx_buf[0] = addr;
    mb_tx_buf[1] = MB_FC_READ_HOLDING_REG;
    mb_tx_buf[2] = (uint8_t)(count * 2);
	
	uint16_t v = 0;
    for (uint16_t i = 0; i < count; i++)
    {
		if(start < 0x0700)
		{
			v = Modbus_RegMap[start + i];
		}
		else
		{
			v= p_FMC_Latched[start-0x0700+i];
		}
        mb_tx_buf[3 + i * 2]     = (uint8_t)((v >> 8) & 0xFF);
        mb_tx_buf[4 + i * 2]     = (uint8_t)(v & 0xFF);
    }
    Modbus_SendWithCRC(3 + count * 2);
}

/* ================= 功能码 0x06：写单个寄存器 ================= */
static void Handle_WriteSingleReg(uint8_t addr, const uint8_t *frame, uint16_t len)
{
    /* 请求格式：addr fc | regH regL | valH valL | crcL crcH   （共 8 字节） */
    if (len != 8)
    {
        Modbus_SendException(addr, MB_FC_WRITE_SINGLE_REG, MB_EX_ILLEGAL_VALUE);
        return;
    }

    uint16_t regaddr = ((uint16_t)frame[2] << 8) | frame[3];
    uint16_t value   = ((uint16_t)frame[4] << 8) | frame[5];

    if (regaddr >= MAX_REG_ADDR)
    {
        Modbus_SendException(addr, MB_FC_WRITE_SINGLE_REG, MB_EX_ILLEGAL_ADDRESS);
        return;
    }

    /* 按块定义表做校验 */
    {
        uint8_t v = Modbus_ValidateRegWrite(regaddr, value);
        if (v == MB_VALIDATE_READONLY)
        {
            Modbus_SendException(addr, MB_FC_WRITE_SINGLE_REG, MB_EX_ILLEGAL_ADDRESS);
            return;
        }
        if (v == MB_VALIDATE_OUT_OF_RANGE)
        {
            Modbus_SendException(addr, MB_FC_WRITE_SINGLE_REG, MB_EX_ILLEGAL_VALUE);
            return;
        }
    }
    {
        /* 修改 Modbus_RegMap 或 FMC 前，必须先通过业务层校验。 */
        uint8_t r = MB_Power_PreWrite(regaddr, &value, 1U);
        if (r != 0U)
        {
            Modbus_SendException(addr, MB_FC_WRITE_SINGLE_REG, r);
            return;
        }
    }

    /* 提交时会更新 Modbus 缓存、GPIO 副作用、FMC 命令和命令序号。 */
    MB_Power_CommitWrite(regaddr, &value, 1U);

    /* 响应：与请求前 6 字节相同 */
    memcpy(mb_tx_buf, frame, 6);
    Modbus_SendWithCRC(6);
}

/* ================= 功能码 0x10：写多个寄存器 ================= */
static void Handle_WriteMultiReg(uint8_t addr, const uint8_t *frame, uint16_t len)
{
    /* 请求：addr fc | regH regL | cntH cntL | bytes | [data...] | crcL crcH */
    if (len < 9)
    {
        Modbus_SendException(addr, MB_FC_WRITE_MULTI_REG, MB_EX_ILLEGAL_VALUE);
        return;
    }

    uint16_t start = ((uint16_t)frame[2] << 8) | frame[3];
    uint16_t count = ((uint16_t)frame[4] << 8) | frame[5];
    uint8_t  bytes = frame[6];

    if (count == 0 || count > 123 || bytes != count * 2)
    {
        Modbus_SendException(addr, MB_FC_WRITE_MULTI_REG, MB_EX_ILLEGAL_VALUE);
        return;
    }
    if (len != (uint16_t)(7 + bytes + 2))    /* 头7 + data + crc2 */
    {
        Modbus_SendException(addr, MB_FC_WRITE_MULTI_REG, MB_EX_ILLEGAL_VALUE);
        return;
    }
    if ((uint32_t)start + count > MAX_REG_ADDR)
    {
        Modbus_SendException(addr, MB_FC_WRITE_MULTI_REG, MB_EX_ILLEGAL_ADDRESS);
        return;
    }

    /* 先全部做范围检查，任何一个非法整个请求拒绝（Modbus 标准做法） */
    for (uint16_t i = 0; i < count; i++)
    {
        uint16_t v = ((uint16_t)frame[7 + i * 2] << 8) | frame[8 + i * 2];
        uint8_t r = Modbus_ValidateRegWrite(start + i, v);
        if (r == MB_VALIDATE_READONLY)
        {
            Modbus_SendException(addr, MB_FC_WRITE_MULTI_REG, MB_EX_ILLEGAL_ADDRESS);
            return;
        }
        if (r == MB_VALIDATE_OUT_OF_RANGE)
        {
            Modbus_SendException(addr, MB_FC_WRITE_MULTI_REG, MB_EX_ILLEGAL_VALUE);
            return;
        }
    }

    {
        uint16_t values[123];
        for (uint16_t i = 0; i < count; i++)
        {
            values[i] = ((uint16_t)frame[7 + i * 2] << 8) | frame[8 + i * 2];
        }

        {
            /* 多寄存器写入先整包校验，任何一个寄存器都不提前提交。 */
            uint8_t r = MB_Power_PreWrite(start, values, count);
            if (r != 0U)
            {
                Modbus_SendException(addr, MB_FC_WRITE_MULTI_REG, r);
                return;
            }
        }

        /* 将已校验通过的整包写请求作为一次业务操作提交。 */
        MB_Power_CommitWrite(start, values, count);
    }

    /* 响应：addr fc | regH regL | cntH cntL | crcL crcH */
    mb_tx_buf[0] = addr;
    mb_tx_buf[1] = MB_FC_WRITE_MULTI_REG;
    mb_tx_buf[2] = (uint8_t)((start >> 8) & 0xFF);
    mb_tx_buf[3] = (uint8_t)(start & 0xFF);
    mb_tx_buf[4] = (uint8_t)((count >> 8) & 0xFF);
    mb_tx_buf[5] = (uint8_t)(count & 0xFF);
    Modbus_SendWithCRC(6);
}

/* ================= 主循环轮询 ================= */
void Modbus_Poll(void)
{
    uint16_t len = 0U;
    uint8_t addr = 0U;
    uint16_t recv_crc = 0U;
    uint16_t calc_crc = 0U;
    uint8_t fc = 0U;

    if (!mb_frame_ready) return;

    /* RX 缓冲溢出（超长帧或噪声灌爆缓冲）：整帧丢弃 */
    if (mb_rx_overrun) goto done;

    len = mb_rx_len;

    /* 帧至少 4 字节：addr + fc + crcL + crcH */
    if (len < 4) goto done;

    /* 地址过滤：0x00 广播；本机地址要处理；其它地址丢弃 */
    addr = mb_rx_buf[0];
    if (addr != MODBUS_SLAVE_ADDR && addr != 0x00) goto done;

    /* CRC 校验 */
    recv_crc = (uint16_t)mb_rx_buf[len - 2] | ((uint16_t)mb_rx_buf[len - 1] << 8);
    calc_crc = Modbus_CRC16(mb_rx_buf, len - 2);
    if (recv_crc != calc_crc) goto done;

    /* 广播不应答 */
    if (addr == 0x00) goto done;

    fc = mb_rx_buf[1];
    switch (fc)
    {
    case MB_FC_READ_COILS:
    case MB_FC_READ_DISC_INPUTS:
        Handle_ReadBitArea(addr, fc, mb_rx_buf, len);
        break;
    case MB_FC_READ_HOLDING_REG:  Handle_ReadHoldingReg(addr, mb_rx_buf, len);  break;
    case MB_FC_READ_INPUT_REG:    Handle_ReadInputReg(addr, mb_rx_buf, len);    break;
    case MB_FC_WRITE_SINGLE_COIL: Handle_WriteSingleCoil(addr, mb_rx_buf, len); break;
    case MB_FC_WRITE_SINGLE_REG:  Handle_WriteSingleReg(addr, mb_rx_buf, len);  break;
    case MB_FC_WRITE_MULTI_COILS: Handle_WriteMultiCoils(addr, mb_rx_buf, len); break;
    case MB_FC_WRITE_MULTI_REG:   Handle_WriteMultiReg(addr, mb_rx_buf, len);   break;
    default:
        Modbus_SendException(addr, fc, MB_EX_ILLEGAL_FUNCTION);
        break;
    }

done:
    Modbus_ResetRxState();
}
