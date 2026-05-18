"""Diagnostic: try different direction-control states on COM10."""
import serial
import time

PORT = "COM10"
BAUD = 115200

def listen(ser, duration, label):
    print(f"\n[{label}] listen {duration}s ...")
    t0 = time.time()
    buf = bytearray()
    while time.time() - t0 < duration:
        data = ser.read(512)
        if data:
            buf.extend(data)
    print(f"[{label}] got {len(buf)} bytes")
    if buf:
        print("ASCII:", buf.decode("ascii", errors="replace"))
        print("HEX  :", buf.hex(' '))
    return buf

def main():
    # Open with explicit no-flow-control
    ser = serial.Serial()
    ser.port = PORT
    ser.baudrate = BAUD
    ser.bytesize = 8
    ser.parity = 'N'
    ser.stopbits = 1
    ser.timeout = 0.1
    ser.rtscts = False
    ser.dsrdtr = False
    ser.xonxoff = False
    ser.open()
    print(f"[open] {PORT} @ {BAUD} 8N1 no-flow-control")
    print(f"       cts={ser.cts} dsr={ser.dsr} ri={ser.ri} cd={ser.cd}")

    # Variant 1: RTS=False, DTR=False (most adapters: RX mode)
    ser.rts = False
    ser.dtr = False
    time.sleep(0.1)
    listen(ser, 4.0, "Variant 1: RTS=0 DTR=0")

    # Variant 2: RTS=True, DTR=False
    ser.rts = True
    ser.dtr = False
    time.sleep(0.1)
    listen(ser, 4.0, "Variant 2: RTS=1 DTR=0")

    # Variant 3: RTS=False, DTR=True
    ser.rts = False
    ser.dtr = True
    time.sleep(0.1)
    listen(ser, 4.0, "Variant 3: RTS=0 DTR=1")

    # Variant 4: RTS=True, DTR=True
    ser.rts = True
    ser.dtr = True
    time.sleep(0.1)
    listen(ser, 4.0, "Variant 4: RTS=1 DTR=1")

    ser.close()
    print("\n[done]")

if __name__ == "__main__":
    main()
