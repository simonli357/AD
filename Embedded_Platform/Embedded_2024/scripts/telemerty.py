#!/usr/bin/env python3
import serial, struct, binascii, warnings

# ────────────────────────────────────────────────────────────────────────────────
SERIAL_PORT = '/dev/ttyACM0'   # ← your port
BAUD_RATE   = 460800
TIMEOUT     = 0.1              # seconds

SOF         = 0xAA             # start‐of‐frame marker
PT_ENCODER  = 0x01
PT_IMU      = 0x02
PT_COMBINED = 0x03

VERIFY_CRC  = False            # set True to check CRC

# suppress that benign requests warning
warnings.filterwarnings("ignore",
    message=".*RequestsDependencyWarning.*", module="requests")
# ────────────────────────────────────────────────────────────────────────────────

def crc16_ccitt(data: bytes, crc: int = 0xFFFF) -> int:
    """CRC-16-CCITT using binascii.crc_hqx."""
    return binascii.crc_hqx(data, crc)

def read_exact(ser, n):
    buf = bytearray()
    while len(buf) < n:
        chunk = ser.read(n - len(buf))
        if not chunk:
            return None
        buf += chunk
    return bytes(buf)

def main():
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=TIMEOUT)
    print(f"Listening on {SERIAL_PORT}@{BAUD_RATE} baud (CRC={'ON' if VERIFY_CRC else 'OFF'})")

    while True:
        # 1) sync on SOF
        b = ser.read(1)
        if not b or b[0] != SOF:
            continue

        # 2) header: [Type][Len]
        hdr = read_exact(ser, 2)
        if not hdr:
            continue
        pkt_type, length = struct.unpack('<BB', hdr)

        # 3) payload + CRC
        payload  = read_exact(ser, length)
        crc_bytes= read_exact(ser, 2)
        if payload is None or crc_bytes is None:
            continue
        (crc_recv,) = struct.unpack('<H', crc_bytes)

        # optional CRC check
        if VERIFY_CRC:
            crc_calc = crc16_ccitt(hdr + payload)
            if crc_calc != crc_recv:
                print(f"CRC FAIL type=0x{pkt_type:02X} len={length} "
                      f"calc=0x{crc_calc:04X} recv=0x{crc_recv:04X}")
                continue

        # 4) decode
        if pkt_type == PT_COMBINED and length == 16:
            # CombinedMsg: [0–3]=int32 angle_h, [4–7]=int32 avg_speed_h,
            #              [8–11]=int32 yaw_h,  [12–15]=int32 pitch_h
            angle_h, avg_h, yaw_h, pitch_h = struct.unpack('<iiii', payload)
            angle     = angle_h  / 100.0
            avg_speed = avg_h    / 100.0
            yaw       = yaw_h    / 100.0
            pitch     = pitch_h  / 100.0

            print(f"[COMB] angle={angle:.2f}°  avg_speed={avg_speed:.2f}°/s  "
                  f"YPR={yaw:.2f},{pitch:.2f}")

        elif pkt_type == PT_ENCODER and length == 8:
            # TelemetryMsg-style encoder: <ii> angle_h, speed_h
            angle_h, speed_h = struct.unpack('<ii', payload)
            print(f"[ENC] angle={angle_h/100:.2f}° speed={speed_h/100:.2f}°/s")

        elif pkt_type == PT_IMU and length == 8:
            # TelemetryMsg-style IMU: <ii> yaw_h, pitch_h
            yaw_h, pitch_h = struct.unpack('<ii', payload)
            print(f"[IMU] yaw={yaw_h/100:.2f}° pitch={pitch_h/100:.2f}°")

        else:
            # unknown or mis-framed
            print(f"[?] type=0x{pkt_type:02X} len={length} payload={payload.hex(' ')}")

if __name__ == '__main__':
    main()
