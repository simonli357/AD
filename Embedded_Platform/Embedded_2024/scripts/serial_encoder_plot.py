#!/usr/bin/env python3

import serial
import time
import matplotlib.pyplot as plt

# --------------------------------------------------------------------------
# Command functions (same as in your examples)
# --------------------------------------------------------------------------
def speed(f_velocity):
    return "#{0}:{1:.2f};;\r\n".format(1, f_velocity)

def steer(f_angle):
    return "#{0}:{1:.2f};;\r\n".format(2, f_angle)

def both(f_velocity, f_angle):
    return "#{0}:{1:.2f}:{2:.2f};;\r\n".format(8, f_velocity, f_angle)

def PWM(f_velocity, f_angle):
    return "#{0}:{1:.2f}:{2:.10f};;\r\n".format(10, f_velocity, f_angle)

def Compute(f_velocity, f_angle):
    return "#{0}:{1:.5f}:{2:.5f};;\r\n".format(13, f_velocity, f_angle)

def setPID(f_active, f_proportional, f_integral, f_derivative):
    return "#{0}:{1:.4f}:{2:.4f}:{3:.4f}:{4:.4f};;\r\n".format(
        12, f_active, f_proportional, f_integral, f_derivative
    )

# --------------------------------------------------------------------------
# Separate time & data arrays for each type of incoming line
# --------------------------------------------------------------------------
angle_time  = []
angle_data  = []

speed_time  = []
speed_data  = []

accel_time  = []
accel_data  = []

# --------------------------------------------------------------------------
# Function to read and parse lines from the serial port
# --------------------------------------------------------------------------
def read_and_store_data(serial_port, start_time):
    """
    Reads one line from 'serial_port', parses if it matches our expected
    data pattern, and appends to the appropriate global arrays.
    """
    # Only read if data is waiting
    if serial_port.in_waiting > 0:
        line = serial_port.readline().decode("utf-8", errors="ignore").strip()
        if not line:
            return

        try:
            # Example lines we parse:
            #   "PWM Angle: 135.60°"
            #   "PWM Speed: 0.25°/s"
            #   "PWM Acceleration: -0.00°/s²"

            if line.startswith("PWM Angle:"):
                val_str = line.replace("PWM Angle:", "").replace("°", "").strip()
                val = float(val_str)
                angle_data.append(val)
                angle_time.append(time.time() - start_time)

            elif line.startswith("PWM Speed:"):
                val_str = line.replace("PWM Speed:", "").replace("°/s", "").strip()
                val = float(val_str)
                speed_data.append(val)
                speed_time.append(time.time() - start_time)

            elif line.startswith("PWM Acceleration:"):
                val_str = (line
                           .replace("PWM Acceleration:", "")
                           .replace("°/s²", "")
                           .strip())
                val = float(val_str)
                accel_data.append(val)
                accel_time.append(time.time() - start_time)

        except ValueError as e:
            print(f"Error parsing line: '{line}' - {e}")

# --------------------------------------------------------------------------
# Main routine
# --------------------------------------------------------------------------
def main():
    # Open serial port
    serial_port = serial.Serial('/dev/ttyACM0', baudrate=115200)
    print("Serial port opened for communication.")

    # Store the reference time (we’ll compute elapsed times from here)
    start_time = time.time()

    try:
        # ------------------------------------------------------------------
        # 1) Turn on PID
        # ------------------------------------------------------------------
        serial_port.write("#{0}:{1:.0f};;\r\n".format(4, 1).encode())
        setPID_msg = setPID(1, 1.25, 0.625, 0.15125)
        print("PID ON msg sent:", setPID_msg)
        serial_port.write(setPID_msg.encode())

        # ------------------------------------------------------------------
        # 2) Send some command, e.g., Compute(30.0, 0.0)
        # ------------------------------------------------------------------
        compute_msg = Compute(30.0, 0.0)
        print("Compute msg sent:", compute_msg)
        serial_port.write(compute_msg.encode())

        # ------------------------------------------------------------------
        # 3) Listen for serial lines for 5 seconds
        # ------------------------------------------------------------------
        print("Listening for data (phase 1, 5s)...")
        phase_1_end = start_time + 5.0
        while time.time() < phase_1_end:
            read_and_store_data(serial_port, start_time)

        # compute_msg = Compute(30.0, 0.0)
        # print("Compute msg sent:", compute_msg)
        # serial_port.write(compute_msg.encode())
        
        # print("Listening for data (phase 2, 5s)...")
        # phase_2_end = phase_1_end + 5.0
        # while time.time() < phase_2_end:
        #     read_and_store_data(serial_port, start_time)

        # ------------------------------------------------------------------
        # 6) STOP the car
        # ------------------------------------------------------------------
        stop_msg = Compute(0.0, 0.0)
        print("STOP msg sent:", stop_msg)
        serial_port.write(stop_msg.encode())

        # ------------------------------------------------------------------
        # 7) Turn off PID
        # ------------------------------------------------------------------
        setPID_msg = setPID(0, 1, 0, 0)
        print("PID OFF msg sent:", setPID_msg)
        serial_port.write(setPID_msg.encode())

    finally:
        # Ensure port is closed even if an error occurs
        serial_port.close()
        print("Serial port closed.")

    # ------------------------------------------------------------------
    # PLOT RESULTS
    # ------------------------------------------------------------------
    print("Plotting the data...")

    fig, axs = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
    fig.suptitle("Angle, Speed, Acceleration vs Time", fontsize=16)

    # 1) Angle Plot
    axs[0].plot(angle_time, angle_data, label="Angle")
    axs[0].set_ylabel("Angle (°)")
    axs[0].legend()
    axs[0].grid(True)

    # 2) Speed Plot
    axs[1].plot(speed_time, speed_data, label="Speed")
    axs[1].set_ylabel("Speed (°/s)")
    axs[1].legend()
    axs[1].grid(True)

    # 3) Acceleration Plot
    axs[2].plot(accel_time, accel_data, label="Acceleration")
    axs[2].set_xlabel("Time (s)")
    axs[2].set_ylabel("Acc (°/s²)")
    axs[2].legend()
    axs[2].grid(True)

    # Make everything look good
    plt.tight_layout(rect=[0, 0, 1, 0.96])
    plt.show()

# --------------------------------------------------------------------------
# Entry point
# --------------------------------------------------------------------------
if __name__ == "__main__":
    main()
