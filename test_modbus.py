"""Modbus RTU master test client for STM32 slave on COM10.
Tests 8 function codes: 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x0F, 0x10.
"""
import serial
import time
import struct

PORT = "COM10"
BAUD = 115200
SLAVE = 0x01

# ---------- CRC ----------
def crc16(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            if crc & 1: crc = (crc >> 1) ^ 0xA001
            else:       crc >>= 1
    return crc

def with_crc(pdu: bytes) -> bytes:
    return pdu + struct.pack("<H", crc16(pdu))

# ---------- Request builders ----------
def rd_coils(addr, start, count):
    return with_crc(struct.pack(">BBHH", addr, 0x01, start, count))

def rd_discrete(addr, start, count):
    return with_crc(struct.pack(">BBHH", addr, 0x02, start, count))

def rd_holding(addr, start, count):
    return with_crc(struct.pack(">BBHH", addr, 0x03, start, count))

def rd_input(addr, start, count):
    return with_crc(struct.pack(">BBHH", addr, 0x04, start, count))

def wr_single_coil(addr, coil, on):
    val = 0xFF00 if on else 0x0000
    return with_crc(struct.pack(">BBHH", addr, 0x05, coil, val))

def wr_single_reg(addr, reg, value):
    return with_crc(struct.pack(">BBHH", addr, 0x06, reg, value))

def wr_multi_coils(addr, start, bits):
    count = len(bits)
    byte_count = (count + 7) // 8
    data = bytearray(byte_count)
    for i, b in enumerate(bits):
        if b: data[i // 8] |= (1 << (i % 8))
    pdu = struct.pack(">BBHHB", addr, 0x0F, start, count, byte_count) + bytes(data)
    return with_crc(pdu)

def wr_multi_regs(addr, start, values):
    count = len(values)
    pdu = struct.pack(">BBHHB", addr, 0x10, start, count, count * 2)
    for v in values: pdu += struct.pack(">H", v)
    return with_crc(pdu)

# ---------- IO ----------
def txrx(ser, req, label):
    ser.reset_input_buffer()
    ser.write(req); ser.flush()
    t0 = time.time()
    buf = bytearray()
    while time.time() - t0 < 0.5:
        buf.extend(ser.read(256))
        if buf and time.time() - t0 > 0.1: break
    time.sleep(0.05); buf.extend(ser.read(512))
    ok = False
    if buf and len(buf) >= 4:
        rx_crc = buf[-2] | (buf[-1] << 8)
        ok = (rx_crc == crc16(bytes(buf[:-2])))
    print(f"\n[{label}]")
    print(f"  TX: {req.hex(' ')}")
    print(f"  RX: {buf.hex(' ') if buf else '(no response)'}  {'CRC OK' if ok else 'CRC FAIL' if buf else ''}")
    return bytes(buf), ok

def check(cond, msg):
    print(f"  {'PASS' if cond else 'FAIL'}: {msg}")
    return cond

# ---------- Tests ----------
def main():
    ser = serial.Serial(PORT, BAUD, bytesize=8, parity='N', stopbits=1, timeout=0.1)
    print(f"[open] {PORT} @ {BAUD} 8N1, slave={SLAVE}")

    passes, fails = 0, 0
    def tally(ok):
        nonlocal passes, fails
        if ok: passes += 1
        else:  fails  += 1

    # ========= 寄存器 =========
    # T1: 读保持寄存器 0x23, 0x24 (预置 0x1234, 0xDCBA)
    rsp, _ = txrx(ser, rd_holding(SLAVE, 0x0023, 2), "T1: 0x03 read holding 0x23..0x24")
    ok = (len(rsp) >= 9 and rsp[1] == 0x03 and rsp[3:7].hex() == '1234dcba')
    tally(check(ok, "reg[0x23]=0x1234, reg[0x24]=0xDCBA"))

    # T2: 读输入寄存器 0x23, 0x24 (本从机与保持寄存器同源，应该值一样)
    rsp, _ = txrx(ser, rd_input(SLAVE, 0x0023, 2), "T2: 0x04 read input 0x23..0x24")
    ok = (len(rsp) >= 9 and rsp[1] == 0x04 and rsp[3:7].hex() == '1234dcba')
    tally(check(ok, "input reg 0x23=0x1234, 0x24=0xDCBA"))

    # T3: 写单个 → 回读
    txrx(ser, wr_single_reg(SLAVE, 0x0100, 0xBEEF), "T3a: 0x06 write single reg 0x0100=0xBEEF")
    rsp, _ = txrx(ser, rd_holding(SLAVE, 0x0100, 1), "T3b: 读回 0x0100")
    ok = (len(rsp) >= 7 and ((rsp[3] << 8) | rsp[4]) == 0xBEEF)
    tally(check(ok, "reg[0x0100]=0xBEEF 写回一致"))

    # T4: 批量写 → 批量读
    txrx(ser, wr_multi_regs(SLAVE, 0x0200, [0xAAAA, 0xBBBB, 0xCCCC]),
         "T4a: 0x10 write multi reg 0x0200..0x0202")
    rsp, _ = txrx(ser, rd_holding(SLAVE, 0x0200, 3), "T4b: 读回 0x0200..0x0202")
    ok = (len(rsp) >= 11 and rsp[3:9].hex() == 'aaaabbbbcccc')
    tally(check(ok, "批量写读一致"))

    # ========= 线圈 =========
    # 预置：Coil[0..7] = 0x5A = 01011010, Coil[8..15] = 0xA5 = 10100101
    # T5: 读线圈 0..15
    rsp, _ = txrx(ser, rd_coils(SLAVE, 0, 16), "T5: 0x01 read coils 0..15 (预置 5A A5)")
    ok = (len(rsp) >= 7 and rsp[1] == 0x01 and rsp[2] == 2
          and rsp[3] == 0x5A and rsp[4] == 0xA5)
    tally(check(ok, "coils[0..15] 返回 5A A5"))

    # T6: 读离散输入 0..15（和线圈共用同一位图，结果应该一样）
    rsp, _ = txrx(ser, rd_discrete(SLAVE, 0, 16), "T6: 0x02 read discrete inputs 0..15")
    ok = (len(rsp) >= 7 and rsp[1] == 0x02 and rsp[3] == 0x5A and rsp[4] == 0xA5)
    tally(check(ok, "discrete inputs[0..15] 返回 5A A5"))

    # T7: 写单个线圈 ON → 回读
    txrx(ser, wr_single_coil(SLAVE, 20, True),  "T7a: 0x05 write coil 20 = ON")
    rsp, _ = txrx(ser, rd_coils(SLAVE, 20, 1), "T7b: 读回 coil 20")
    ok = (len(rsp) >= 6 and (rsp[3] & 0x01) == 0x01)
    tally(check(ok, "coil[20] = 1"))

    # T8: 写单个线圈 OFF → 回读
    txrx(ser, wr_single_coil(SLAVE, 20, False), "T8a: 0x05 write coil 20 = OFF")
    rsp, _ = txrx(ser, rd_coils(SLAVE, 20, 1), "T8b: 读回 coil 20")
    ok = (len(rsp) >= 6 and (rsp[3] & 0x01) == 0x00)
    tally(check(ok, "coil[20] = 0"))

    # T9: 写单个线圈 值非法（0x1234，不是 0xFF00/0x0000）→ 异常 0x03
    bad = with_crc(struct.pack(">BBHH", SLAVE, 0x05, 30, 0x1234))
    rsp, _ = txrx(ser, bad, "T9: 0x05 非法线圈值 0x1234 (期望异常码 0x03)")
    ok = (len(rsp) >= 5 and rsp[1] == (0x05 | 0x80) and rsp[2] == 0x03)
    tally(check(ok, "返回异常码 0x03 非法数据"))

    # T10: 写多个线圈：从 coil 100 写 10 个，模式 = 1010101010 = 0x55, 0x01
    txrx(ser, wr_multi_coils(SLAVE, 100, [1,0,1,0,1,0,1,0,1,0]),
         "T10a: 0x0F write multi coils 100..109 = 1010101010")
    rsp, _ = txrx(ser, rd_coils(SLAVE, 100, 10), "T10b: 读回 coils 100..109")
    # 低 8 bit 应为 0b01010101=0x55, 高 2 bit 应为 0b01=0x01
    ok = (len(rsp) >= 7 and rsp[2] == 2 and rsp[3] == 0x55 and (rsp[4] & 0x03) == 0x01)
    tally(check(ok, "coils[100..109] 写读一致"))

    # T11: 非法地址 — 读 0xFFFF 起的保持寄存器
    rsp, _ = txrx(ser, rd_holding(SLAVE, 0xFFFF, 1), "T11: 读越界地址 0xFFFF (期望异常码 0x02)")
    ok = (len(rsp) >= 5 and rsp[1] == (0x03 | 0x80) and rsp[2] == 0x02)
    tally(check(ok, "返回异常码 0x02 非法地址"))

    # T12: 非法功能码
    bad = with_crc(bytes([SLAVE, 0x99, 0x00, 0x00, 0x00, 0x01]))
    rsp, _ = txrx(ser, bad, "T12: 非法功能码 0x99 (期望异常码 0x01)")
    ok = (len(rsp) >= 5 and rsp[1] == (0x99 | 0x80) and rsp[2] == 0x01)
    tally(check(ok, "返回异常码 0x01 非法功能码"))

    ser.close()
    print(f"\n========== {passes} PASS / {fails} FAIL ==========")

if __name__ == "__main__":
    main()
