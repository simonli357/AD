#!/usr/bin/env python3

import serial
import time
import matplotlib
# matplotlib.use('Agg')  # Use a non-interactive backend
import matplotlib.pyplot as plt


def compute(f_velocity, f_angle):
    return "#{0}:{1:.2f}:{2:.2f};;\r\n".format(13, f_velocity, f_angle)

def definite_c(f_velocity, f_angle):
    return "#{0}:{1:.2f}:{2:.2f};;\r\n".format(11, f_velocity, f_angle)

def setPID(f_active, f_proportional, f_integral, f_derivative):
    return "#{0}:{1:.4f}:{2:.4f}:{3:.4f}:{4:.4f};;\r\n".format(12, f_active, f_proportional, f_integral, f_derivative)

def read_file(file_path):
    """Reads a file and extracts velocity and angle values."""
    data = []
    try:
        with open(file_path, 'r') as file:
            for line in file:
                try:
                    velocity, angle = map(float, line.strip().split(','))
                    data.append((velocity, angle))
                except ValueError:
                    print(f"Skipping invalid line: {line.strip()}")
    except FileNotFoundError:
        print("File not found!")
    return data


def send_over_serial(file_path, serial_port):
    """Reads data from a file and sends it over the serial port at 10Hz."""
    try:
        ser = serial.Serial(serial_port, baudrate=460800)
        print("Serial port opened for communication.")

        # Set the PID
        # setPID_msg = setPID(1, 1.25, 0.625, 0.15125)
        # print("PID ON msg sent:", setPID_msg)
        # ser.write(setPID_msg.encode())

        start_time = time.time  # Get the start time

        data = read_file(file_path)
        with open("sent_messages.txt", "a") as log_file:
            for velocity, angle in data:
                message = compute(velocity, angle)
                ser.write(message.encode('utf-8'))
                # print(f"Sent: {message.strip()}")
                time.sleep(0.1)  # Sending at 10Hz

    except serial.SerialException as e:
        print(f"Serial error: {e}")

    except Exception as e:
        print(f"Unexpected error: {e}")

    finally:
        if 'ser' in locals() and ser.is_open:

            # Turn off the PID
            setPID_msg = setPID(0, 10, 5, 0.5)
            print("PID OFF msg sent:", setPID_msg)
            ser.write(setPID_msg.encode())

            # Stop the car
            pwm_msg = definite_c(0.0, 0.0)
            print("STOP msg sent:", pwm_msg)
            ser.write(pwm_msg.encode())


            ser.close()
            print("Serial port closed.")

if __name__ == "__main__":
    file_path = "/home/malo/Documents/Simulator/src/example/src/left_turn_commands0202.txt"  # Change this to your file's path
    serial_port = "/dev/ttyACM0"  # Adjust as needed
    send_over_serial(file_path, serial_port)



