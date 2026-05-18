"""Modbus test for 自主中频电源控制板 交互数据表
"""
import serial, time, struct

PORT = "COM10"
BAUD = 115200
SLAVE = 0x01

# ---------- 寄存器地址（对应 modbus_regs.h） ----------
REG_UNIT_TYPE              = 0x0000
REG_UNIT_POWER_LIMIT_HI    = 0x0023
REG_UNIT_POWER_LIMIT_LO    = 0x0024
REG_UNIT_VOLTAGE_LIMIT     = 0x0025
REG_UNIT_CURRENT_LIMIT     = 0x0026

REG_OUTPUT_ONOFF           = 0x0100
REG_REGULATION_MODE        = 0x0101
REG_SETPOINT_HI            = 0x0102
REG_SETPOINT_LO            = 0x0103
REG_CONTROL_MODE           = 0x0104

REG_USER_POWER_LIMIT_HI    = 0x0140
REG_USER_VOLTAGE_LIMIT     = 0x0142
REG_USER_CURRENT_LIMIT     = 0x0143
REG_IGNITION_SWITCH        = 0x0144

REG_OUTPUT_FREQUENCY       = 0x0165
REG_PULSE_DUTY_CYCLE       = 0x0166

REG_RAMP_STEP              = 0x0191
REG_RAMP_TIME_MS           = 0x0192

# ---------- 预期出厂值 ----------
UNIT_POWER_LIMIT_W         = 15000
UNIT_VOLTAGE_LIMIT_V       = 600
UNIT_CURRENT_LIMIT_01A     = 1500

# ---------- CRC ----------
def crc16(data: bytes) -> int:
    c = 0xFFFF
    for b in data:
        c ^= b
        for _ in range(8):
            c = (c >> 1) ^ 0xA001 if c & 1 else c >> 1
    return c

def wc(pdu): return pdu + struct.pack("<H", crc16(pdu))

def rd_holding(addr, start, count):
    return wc(struct.pack(">BBHH", addr, 0x03, start, count))
def wr_single(addr, reg, val):
    return wc(struct.pack(">BBHH", addr, 0x06, reg, val))
def wr_multi(addr, start, vals):
    pdu = struct.pack(">BBHHB", addr, 0x10, start, len(vals), len(vals)*2)
    for v in vals: pdu += struct.pack(">H", v)
    return wc(pdu)

def txrx(ser, req, label=""):
    ser.reset_input_buffer()
    ser.write(req); ser.flush()
    time.sleep(0.15)
    buf = ser.read(512)
    ok_crc = len(buf) >= 4 and crc16(bytes(buf[:-2])) == (buf[-2] | (buf[-1]<<8))
    if label:
        print(f"\n[{label}]")
        print(f"  TX: {req.hex(' ')}")
        print(f"  RX: {buf.hex(' ') if buf else '(no response)'}")
    return bytes(buf), ok_crc

def rd_word(ser, reg, label=""):
    rsp, _ = txrx(ser, rd_holding(SLAVE, reg, 1), label)
    if len(rsp) < 7 or rsp[1] != 0x03: return None
    return (rsp[3]<<8) | rsp[4]

def rd_dword(ser, reg, label=""):
    rsp, _ = txrx(ser, rd_holding(SLAVE, reg, 2), label)
    if len(rsp) < 9 or rsp[1] != 0x03: return None
    return (rsp[3]<<24) | (rsp[4]<<16) | (rsp[5]<<8) | rsp[6]

def is_exception(rsp, expected_fc, expected_code):
    return (len(rsp) >= 5 and rsp[1] == (expected_fc | 0x80) and rsp[2] == expected_code)

def check(ok, msg):
    print(f"  {'PASS' if ok else 'FAIL'}: {msg}")
    return ok

# ---------- main ----------
def main():
    ser = serial.Serial(PORT, BAUD, 8, 'N', 1, timeout=0.1)
    print(f"[open] {PORT} @ {BAUD} slave={SLAVE}")
    p, f = 0, 0
    def tally(ok):
        nonlocal p,f
        if ok: p+=1
        else:  f+=1

    # ===== 工厂出厂值验证 =====
    print("\n########## 读取出厂值 ##########")

    v = rd_dword(ser, REG_UNIT_POWER_LIMIT_HI, "T1: Unit power limit (32-bit @ 0x23)")
    tally(check(v == UNIT_POWER_LIMIT_W, f"Unit power limit = {UNIT_POWER_LIMIT_W} W (实际={v})"))

    v = rd_word(ser, REG_UNIT_VOLTAGE_LIMIT, "T2: Unit voltage limit (0x25)")
    tally(check(v == UNIT_VOLTAGE_LIMIT_V, f"Unit voltage limit = {UNIT_VOLTAGE_LIMIT_V} V (实际={v})"))

    v = rd_word(ser, REG_UNIT_CURRENT_LIMIT, "T3: Unit current limit (0x26)")
    tally(check(v == UNIT_CURRENT_LIMIT_01A, f"Unit current limit = {UNIT_CURRENT_LIMIT_01A} (0.1A) (实际={v})"))

    v = rd_word(ser, REG_OUTPUT_ONOFF, "T4: output on/off 默认")
    tally(check(v == 0, f"output 默认=0 (实际={v})"))

    v = rd_word(ser, REG_REGULATION_MODE, "T5: regulation mode 默认")
    tally(check(v == 6, f"regulation mode 默认=6 功率 (实际={v})"))

    v = rd_word(ser, REG_CONTROL_MODE, "T6: control mode 默认")
    tally(check(v == 2, f"control mode 默认=2 主机 (实际={v})"))

    v = rd_word(ser, REG_PULSE_DUTY_CYCLE, "T7: pulse duty cycle 默认")
    tally(check(v == 50, f"duty cycle 默认=50 (实际={v})"))

    v = rd_word(ser, REG_RAMP_TIME_MS, "T8: ramp time 默认")
    tally(check(v == 1000, f"ramp time 默认=1000 ms (实际={v})"))

    # ===== 合法写操作 =====
    print("\n########## 合法写入 ##########")

    txrx(ser, wr_single(SLAVE, REG_OUTPUT_ONOFF, 1), "T9a: 开功率")
    v = rd_word(ser, REG_OUTPUT_ONOFF, "T9b: 读回")
    tally(check(v == 1, f"output on = 1"))

    rsp, _ = txrx(ser, wr_single(SLAVE, REG_OUTPUT_FREQUENCY, 1000),
                  "T9c: output=1 写 output frequency (期望异常 0x03)")
    tally(check(is_exception(rsp, 0x06, 0x03), "output=1 时禁止写输出频率 → 异常 0x03"))

    txrx(ser, wr_single(SLAVE, REG_OUTPUT_ONOFF, 0), "T9d: 关功率")
    txrx(ser, wr_single(SLAVE, REG_OUTPUT_FREQUENCY, 1000),
         "T9e: output=0 写 output frequency")
    v = rd_word(ser, REG_OUTPUT_FREQUENCY, "T9f: 读回 output frequency")
    tally(check(v == 1000, f"output=0 时输出频率可写 (实际={v})"))

    txrx(ser, wr_single(SLAVE, REG_REGULATION_MODE, 7), "T10a: 切到电压模式")
    v = rd_word(ser, REG_REGULATION_MODE, "T10b: 读回")
    tally(check(v == 7, f"regulation mode = 7 电压"))

    txrx(ser, wr_multi(SLAVE, REG_SETPOINT_HI, [0x0001, 0xD4C0]), "T11a: 写 setpoint = 120000")
    v = rd_dword(ser, REG_SETPOINT_HI, "T11b: 读回")
    tally(check(v == 120000, f"setpoint = 120000 (实际={v})"))

    # ===== 非法写：工厂只读区 =====
    print("\n########## 只读区写入测试 ##########")

    rsp, _ = txrx(ser, wr_single(SLAVE, REG_UNIT_POWER_LIMIT_LO, 9999),
                  "T12: 写只读区 Unit power limit (期望异常 0x02)")
    tally(check(is_exception(rsp, 0x06, 0x02), "工厂限值只读 → 异常 0x02"))

    rsp, _ = txrx(ser, wr_single(SLAVE, REG_UNIT_TYPE, 0x4142),
                  "T13: 写只读 Unit Type (期望异常 0x02)")
    tally(check(is_exception(rsp, 0x06, 0x02), "Unit Type 只读 → 异常 0x02"))

    # ===== 非法写：超范围 =====
    print("\n########## 超范围写入测试 ##########")

    rsp, _ = txrx(ser, wr_single(SLAVE, REG_OUTPUT_ONOFF, 99),
                  "T14: output on/off 写 99 (期望异常 0x03)")
    tally(check(is_exception(rsp, 0x06, 0x03), "超范围 99 > max=1 → 异常 0x03"))

    rsp, _ = txrx(ser, wr_single(SLAVE, REG_REGULATION_MODE, 5),
                  "T15: regulation mode 写 5 (min=6, 期望异常 0x03)")
    tally(check(is_exception(rsp, 0x06, 0x03), "超范围 5 < min=6 → 异常 0x03"))

    rsp, _ = txrx(ser, wr_single(SLAVE, REG_CONTROL_MODE, 3),
                  "T16: control mode 写 3 (合法=2/4, 期望异常 0x03)")
    tally(check(is_exception(rsp, 0x06, 0x03), "超范围 3 不在 [2,4] → 异常 0x03"))

    rsp, _ = txrx(ser, wr_single(SLAVE, REG_IGNITION_SETPOINT := 0x0145, 5),
                  "T17: ignition setpoint 写 5 (max=2, 期望异常 0x03)")
    tally(check(is_exception(rsp, 0x06, 0x03), "超范围 5 > max=2 → 异常 0x03"))

    rsp, _ = txrx(ser, wr_single(SLAVE, REG_PULSE_DUTY_CYCLE, 200),
                  "T18: duty cycle 写 200 (max=100, 期望异常 0x03)")
    tally(check(is_exception(rsp, 0x06, 0x03), "超范围 200 > max=100 → 异常 0x03"))

    # ===== 批量写跨区检查 =====
    print("\n########## 批量写合法/非法 ##########")

    # 合法批量写：都是用户可写
    txrx(ser, wr_multi(SLAVE, REG_OUTPUT_ONOFF, [0, 6]), "T19a: 合法批量写")
    v0 = rd_word(ser, REG_OUTPUT_ONOFF)
    v1 = rd_word(ser, REG_REGULATION_MODE)
    tally(check(v0 == 0 and v1 == 6, f"output=0, mode=6 (实际={v0},{v1})"))

    # 非法批量写：跨区包含只读 (0x0020 区)
    rsp, _ = txrx(ser, wr_multi(SLAVE, 0x0020, [1, 2, 3, 4, 5, 6]),
                  "T20: 跨只读区批量写 (期望异常 0x02)")
    tally(check(is_exception(rsp, 0x10, 0x02), "跨只读区 → 异常 0x02"))

    ser.close()
    print(f"\n========== {p} PASS / {f} FAIL ==========")

if __name__ == "__main__":
    main()
