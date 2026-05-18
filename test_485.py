"""USART2 RS485 test over COM10."""
import serial
import time
import sys

PORT = "COM10"
BAUD = 115200

def main():
    ser = serial.Serial(PORT, BAUD, bytesize=8, parity='N', stopbits=1, timeout=0.1)
    print(f"[open] {PORT} @ {BAUD} 8N1")

    # ---------- Phase 1: passive listen 3.5s for welcome + heartbeats ----------
    print("\n[Phase 1] listening 3.5s for welcome + heartbeat ...")
    t0 = time.time()
    buf = bytearray()
    while time.time() - t0 < 3.5:
        data = ser.read(256)
        if data:
            buf.extend(data)
    print(f"[Phase 1] got {len(buf)} bytes:")
    try:
        print(buf.decode("ascii", errors="replace"))
    except Exception as e:
        print(f"<decode error {e}>")
    print(f"[Phase 1] HEX: {buf.hex(' ')}")

    # ---------- Phase 2: send 'A' and check echo ----------
    print("\n[Phase 2] send 'A' (0x41), wait 0.5s for echo ...")
    ser.reset_input_buffer()
    ser.write(b"A")
    ser.flush()
    time.sleep(0.5)
    resp = ser.read(512)
    print(f"[Phase 2] got {len(resp)} bytes:")
    print(resp.decode("ascii", errors="replace"))
    print(f"[Phase 2] HEX: {resp.hex(' ')}")
    if b"RX:0x41" in resp:
        print("[Phase 2] PASS: echo of 'A' detected")
    else:
        print("[Phase 2] FAIL: did not see 'RX:0x41'")

    # ---------- Phase 3: send multiple bytes ----------
    print("\n[Phase 3] send 'HELLO', wait 1.0s for echoes ...")
    ser.reset_input_buffer()
    ser.write(b"HELLO")
    ser.flush()
    time.sleep(1.0)
    resp = ser.read(1024)
    print(f"[Phase 3] got {len(resp)} bytes:")
    print(resp.decode("ascii", errors="replace"))

    # ---------- Phase 4: heartbeat cadence check ----------
    print("\n[Phase 4] measure heartbeat cadence (~5s) ...")
    ser.reset_input_buffer()
    t0 = time.time()
    line = bytearray()
    hb_times = []
    while time.time() - t0 < 5.5:
        b = ser.read(1)
        if not b:
            continue
        line.append(b[0])
        if b == b"\n":
            s = line.decode("ascii", errors="replace").strip()
            if s.startswith("HB "):
                hb_times.append(time.time())
                print(f"  [{time.time()-t0:5.2f}s] {s}")
            line.clear()
    if len(hb_times) >= 2:
        deltas = [hb_times[i+1]-hb_times[i] for i in range(len(hb_times)-1)]
        print(f"[Phase 4] {len(hb_times)} heartbeats, intervals: "
              + ", ".join(f"{d:.3f}s" for d in deltas))
    else:
        print(f"[Phase 4] only {len(hb_times)} heartbeat(s) captured")

    ser.close()
    print("\n[done]")

if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        print(f"ERROR: {e}")
        sys.exit(1)
