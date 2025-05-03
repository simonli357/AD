#!/usr/bin/env python3

import serial
import time

def read_serial(port, baudrate, output_file):
    try:
        ser = serial.Serial(port, baudrate, timeout=1)
        print(f"Listening on {port} at {baudrate} baud...")
        
        with open(output_file, "a") as file:
            while True:
                if ser.in_waiting > 0:
                    line = ser.readline().decode('utf-8', errors='ignore').strip()
                    print(line)  # Print to console
                    file.write(line + "\n")  # Save to file
    except serial.SerialException as e:
        print(f"Serial error: {e}")
    except KeyboardInterrupt:
        print("Logging stopped by user.")
    finally:
        if 'ser' in locals() and ser.is_open:
            ser.close()
            print("Serial port closed.")

def compute(f_velocity, f_angle):
    return "#{0}:{1:.2f}:{2:.2f};;\r\n".format(13, f_velocity, f_angle)

if __name__ == "__main__":
    serial_port_name = "/dev/ttyACM0"  # Adjust as needed
    baud_rate = 115200
    output_filename = "received_serial_data.txt"

    try:
        # Open the serial port
        serial_port = serial.Serial(serial_port_name, baud_rate, timeout=1)
        print(f"Opened serial port {serial_port_name} at {baud_rate} baud.")

        # Send the initial message
        init_msg = compute(10.0, 0.0)
        print("Sending compute():", init_msg.strip())
        serial_port.write(init_msg.encode())

        # Start reading from the serial port
        read_serial(serial_port_name, baud_rate, output_filename)
    except serial.SerialException as e:
        print(f"Failed to open serial port: {e}")
    finally:
        if 'serial_port' in locals() and serial_port.is_open:
            serial_port.close()
            print("Serial port closed.")