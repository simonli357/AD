import serial
import threading
import time

def read_from_port(ser):
    """Continuously read lines from the serial port and print them."""
    while True:
        try:
            line = ser.readline().decode('utf-8', errors='replace')
            if line:
                print(f"[STM32] {line.strip()}")
        except Exception as e:
            print(f"[Error reading] {e}")
            break

def send_commands(ser, velocity, angle, use_pid=True, rate_hz=10):
    """Continuously send formatted commands at the specified rate."""
    interval = 1.0 / rate_hz
    number = "11"
    while True:
        formatted = f"#{number}:{velocity * 100:.2f}:{angle:.2f};;\r\n"
        try:
            ser.write(formatted.encode('utf-8'))
            print(f"[Sent] {formatted.strip()}")
        except Exception as e:
            print(f"[Error writing] {e}")
            break
        time.sleep(interval)

def send_commands2(ser, velocity, use_pid=True, rate_hz=10):
    """Continuously send formatted commands at the specified rate with cycling steering angle."""
    interval = 1.0 / rate_hz
    number = "11" if use_pid else "13"

    angles = [0, 20, -20]
    index = 0

    while True:
        angle = angles[index]
        formatted = f"#{number}:{velocity * 100:.2f}:{angle:.2f};;\r\n"
        try:
            ser.write(formatted.encode('utf-8'))
            print(f"[Sent] {formatted.strip()}")
        except Exception as e:
            print(f"[Error writing] {e}")
            break

        index = (index + 1) % len(angles)
        time.sleep(interval)
def main():
    port = "/dev/ttyACM0"  # Change as needed
    baud = 460800

    try:
        ser = serial.Serial(port, baudrate=baud, timeout=1)
    except serial.SerialException as e:
        print(f"Failed to open {port}: {e}")
        return

    print(f"Opened {port} at {baud} baud.")

    reader_thread = threading.Thread(target=read_from_port, args=(ser,), daemon=True)
    reader_thread.start()

    send_commands(ser, velocity=0.0, angle=-0.0, use_pid=True)

if __name__ == "__main__":
    main()
