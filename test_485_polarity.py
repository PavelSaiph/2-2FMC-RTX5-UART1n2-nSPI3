"""Listen to USART2 polarity diagnostic for 8 seconds."""
import serial
import time

PORT = "COM10"
BAUD = 115200

ser = serial.Serial(PORT, BAUD, bytesize=8, parity='N', stopbits=1, timeout=0.1)
print(f"[open] {PORT} @ {BAUD} 8N1")
print("[listen] 8 seconds ...\n")

t0 = time.time()
buf = bytearray()
while time.time() - t0 < 8.0:
    data = ser.read(512)
    if data:
        buf.extend(data)

print(f"[result] {len(buf)} bytes\n")
text = buf.decode("ascii", errors="replace")
print("==== ASCII ====")
print(text)
print("==== HEX  ====")
print(buf.hex(' '))

txh_count = text.count("TXH")
txl_count = text.count("TXL")
print(f"\n[count] TXH messages: {txh_count}")
print(f"[count] TXL messages: {txl_count}")

if txh_count and not txl_count:
    print("[verdict] HIGH=TX is correct (original polarity OK)")
elif txl_count and not txh_count:
    print("[verdict] LOW=TX is correct (polarity needs to be inverted)")
elif txh_count and txl_count:
    print("[verdict] both polarities work (auto-direction transceiver)")
else:
    print("[verdict] NO data at all - hardware issue (wiring / power / wrong pin)")

ser.close()
