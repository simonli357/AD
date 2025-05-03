#!/usr/bin/env python3
import serial, struct, binascii, warnings

# ── Configuration ────────────────────────────────────────────────────────────────
SERIAL_PORT   = '/dev/ttyACM0'  # adjust to your port
BAUD_RATE     = 115200
TIMEOUT       = 0.1             # seconds

SOF           = 0xAA            # start‐of‐frame marker
PT_ENCODER    = 0x01
PT_IMU        = 0x02
PT_COMBINED   = 0x03            # new packet type

# Toggle CRC verification
VERIFY_CRC    = False

# suppress that benign warning
warnings.filterwarnings("ignore",
    message=".*RequestsDependencyWarning.*", module="requests")

def crc16_ccitt(data: bytes, crc: int = 0xFFFF) -> int:
    return binascii.crc_hqx(data, crc)

def read_exact(ser, n):
    buf = bytearray()
    while len(buf) < n:
        chunk = ser.read(n - len(buf))
        if not chunk:
            return None
        buf += chunk
    return bytes(buf)

def compute(f_velocity, f_angle):
    return "#{0}:{1:.2f}:{2:.2f};;\r\n".format(13, f_velocity, f_angle)

def PWM(f_velocity, f_angle):
    return "#{0}:{1:.10f}:{2:.10f};;\r\n".format(10, f_velocity, f_angle)

def main():
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=TIMEOUT)
    print(f"Listening on {SERIAL_PORT}@{BAUD_RATE} baud   (CRC={'ON' if VERIFY_CRC else 'OFF'})")

    # (optional) send your init message
    init_msg = PWM(0.06975, 0.0)
    ser.write(init_msg.encode('utf-8'))
    print(f"Sent init message: {init_msg.strip()}")

    while True:
        # 1) wait for start‐of‐frame
        b = ser.read(1)
        if not b or b[0] != SOF:
            continue

        # 2) read packet header
        hdr = read_exact(ser, 2)
        if not hdr:
            continue
        pkt_type, length = struct.unpack('<BB', hdr)

        # 3) read payload + CRC
        payload   = read_exact(ser, length)
        # print("RX BYTES:", payload.hex(" "))
        crc_bytes = read_exact(ser, 2)
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

        # 4) decode based on packet type
        if pkt_type == PT_ENCODER and length == 4:
            # [angle_h][speed_h]
            angle_h, speed_h = struct.unpack('<hh', payload)
            angle     = angle_h / 100.0
            speed     = speed_h / 100.0
            print(f"[ENC] angle = {angle:.2f}°   speed = {speed:.2f}°/s")

        elif pkt_type == PT_IMU and length == 18:
            # 9×int16: yaw,pitch,roll ; ax,ay,az ; gx,gy,gz
            vals = struct.unpack('<9h', payload)
            yaw, pitch, roll = [v/100.0   for v in vals[0:3]]
            ax, ay, az      = [v/1000.0  for v in vals[3:6]]
            gx, gy, gz      = [v/1000.0  for v in vals[6:9]]
            print(f"[IMU] YPR={yaw:6.2f},{pitch:6.2f},{roll:6.2f}  "
                  f"a={ax:5.3f},{ay:5.3f},{az:5.3f}  "
                  f"g={gx:5.3f},{gy:5.3f},{gz:5.3f}")

        elif pkt_type == PT_COMBINED and length == 24:
            # 0–1: angle (int16)
            # 2–5: avg_speed (int32)
            # 6–23: imu (9×int16)
            angle_h, avg_h    = struct.unpack_from('<hi', payload, 0)
            imu_vals          = struct.unpack_from('<9h', payload, 6)

            angle     = angle_h    / 100.0
            avg_speed = avg_h      / 100.0

            yaw, pitch, roll = [v/100.0   for v in imu_vals[0:3]]
            ax,  ay,   az   = [v/1000.0  for v in imu_vals[3:6]]
            gx,  gy,   gz   = [v/1000.0  for v in imu_vals[6:9]]

            print(f"[COMB] angle={angle:.2f}°  avg_speed={avg_speed:.2f}°/s  "
                # f"YPR={yaw:.2f},{pitch:.2f},{roll:.2f}  "
                # f"a={ax:.3f},{ay:.3f},{az:.3f}  "
                # f"g={gx:.3f},{gy:.3f},{gz:.3f}"
                )

if __name__ == '__main__':
    main()
