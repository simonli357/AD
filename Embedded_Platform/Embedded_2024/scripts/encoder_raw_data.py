import serial
import sys

def main():
    # Configuration
    port = '/dev/ttyACM0'
    baud = 115200
    timeout = 1.0

    try:
        # 1. Initialize Serial Connection
        ser = serial.Serial(port, baud, timeout=timeout)
        print(f"--- Connected to {port} at {baud} baud ---")
        print("--- Press Ctrl+C to stop ---\n")

        while True:
            if ser.in_waiting > 0:
                # 2. Read a line from the STM32
                # We use errors='ignore' to prevent crashes on partial bytes
                line = ser.readline().decode('utf-8', errors='ignore').strip()

                if line:
                    # 3. Check if it's our encoder data
                    if line.startswith("@5:") and line.endswith(";;"):
                        # Strip the prefix and suffix to see just the numbers
                        raw_values = line.replace("@5:", "").replace(";;", "")
                        print(f"Encoder Data: {raw_values}")
                    else:
                        # Print other messages (like the "PWM Encoder Initialized" printf)
                        print(f"System Message: {line}")

    except serial.SerialException as e:
        print(f"\n[Error]: Could not open {port}. Is the STM32 plugged in?")
        print(f"Details: {e}")
    except KeyboardInterrupt:
        print("\nStopping reader...")
    finally:
        if 'ser' in locals() and ser.is_open:
            ser.close()
            print("Serial port closed.")

if __name__ == "__main__":
    main()