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

def send_commands(ser, velocity, angle, rate_hz=10):
    """Continuously send formatted commands at the specified rate."""
    interval = 1.0 / rate_hz
    while True:
        msg = f"{velocity:.2f}:{angle:.2f};;\r\n"
        try:
            ser.write(msg.encode('utf-8'))
            print(f"[Sent] {msg.strip()}")
        except Exception as e:
            print(f"[Error writing] {e}")
            break
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

    # Start the reader thread
    reader_thread = threading.Thread(target=read_from_port, args=(ser,), daemon=True)
    reader_thread.start()

    # Start the sender loop (velocity = 0, angle = 0)
    send_commands(ser, velocity=0.0, angle=0.0)

if __name__ == "__main__":
    main()
