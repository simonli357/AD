#!/usr/bin/env python3

import serial
import time
import matplotlib
# matplotlib.use('Agg')  # Use a non-interactive backend
import matplotlib.pyplot as plt

# On old car(2023): speed is in m/s
# Command - 4: Activate (send > 1) and deactivate (send 0) the PID
# Command - 5: Enable (send > 1) and disable (send 0) listening to the encoder (listen on serial port)
# LIMITS: speeds: {-0.5, 0.5} m/s, angles: {-25, 25} degrees

# On new car(2024): speed is in cm/s

# Data storage
time_data = []
yaw_calc_data = []
imu_yaw_data = []
error_data = []
integral_data = []
derivative_data = []
newSteer_data = []
clipSteer_data = []

# Functions to send commands
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
    return "#{0}:{1:.4f}:{2:.4f}:{3:.4f}:{4:.4f};;\r\n".format(12, f_active, f_proportional, f_integral, f_derivative)

def readPlot(serial_port, start_time):
    if serial_port.in_waiting > 0:
        line = serial_port.readline().decode('utf-8').strip()
        try:
            if line.startswith("@Y1:"):
                yaw_calc_data.append(float(line[4:]))
            elif line.startswith("@Y2:"):
                imu_yaw_data.append(float(line[4:]))
            elif line.startswith("@E1:"):
                error_data.append(float(line[4:]))
            elif line.startswith("@E2:"):
                integral_data.append(float(line[4:]))
            elif line.startswith("@Y4:"):
                newSteer_data.append(float(line[4:]))
            elif line.startswith("@Y5:"):
                clipSteer_data.append(float(line[4:]))
            elif line.startswith("@E3:"):
                derivative_data.append(float(line[4:]))
                time_data.append(time.time() - start_time)
        except ValueError as e:
            print(f"Error parsing line: {line} - {e}")


def main():
    # Serial port configuration
    serial_port = serial.Serial('/dev/ttyACM0', baudrate=115200)
    print("Serial port opened for communication.")

    try:
        # Send commands
        serial_port.write("#{0}:{1:.0f};;\r\n".format(4, 1).encode())
        # Tuned parameters : Kp = 1.5, Ki = #, Kd = #;
        setPID_msg = setPID(1, 1.25, 0.625, 0.15125)
        print("PID ON msg sent:", setPID_msg)
        serial_port.write(setPID_msg.encode())

        compute_msg = Compute(30.0, 0.0)
        print("Compute msg sent:", compute_msg)
        serial_port.write(compute_msg.encode())

        print("Listening to serial data...")
        start_time = time.time()

        while time.time() - start_time < 1:  # Collect data for 10 seconds
            readPlot(serial_port, start_time)

        compute_msg = Compute(30.0, -14.0)
        print("Compute msg sent:", compute_msg)
        serial_port.write(compute_msg.encode())

        while time.time() - start_time < 2:  # Collect data from 10 to 20 seconds
            readPlot(serial_port, start_time)

        compute_msg = Compute(30.0, 14.0) 
        serial_port.write(compute_msg.encode())
        print("Compute msg sent:", compute_msg)
        while time.time() - start_time < 4:  # Collect data for 10 seconds
            readPlot(serial_port, start_time)

        compute_msg = Compute(30.0, -14.0)
        serial_port.write(compute_msg.encode())
        print("Compute msg sent:", compute_msg)
        while time.time() - start_time < 5:  # Collect data for 10 seconds
            readPlot(serial_port, start_time) 

        pwm_msg = Compute(0.0, 0.0)
        print("STOP msg sent:", pwm_msg)
        serial_port.write(pwm_msg.encode())

        setPID_msg = setPID(0, 1, 0, 0)
        print("PID OFF msg sent:", setPID_msg)
        serial_port.write(setPID_msg.encode())

    finally:
        serial_port.close()
        print("Serial port closed.")

    # Plot IMU and Error Data
    print("Plotting the data...")

    fig, axs = plt.subplots(2, 1, figsize=(10, 8), sharex=True)  # 2 rows, 1 column of subplots
    fig.suptitle("Serial Data Plots", fontsize=16)

    # min_length = min(len(time_data), len(error_data), len(integral_data), len(derivative_data))

    # print("Minimum length:", min_length)   

    # time_data = time_data[:min_length]
    # error_data = error_data[:min_length]
    # integral_data = integral_data[:min_length]
    # derivative_data = derivative_data[:min_length]
    # imu_yaw_data = imu_yaw_data[:min_length]
    # yaw_calc_data = yaw_calc_data[:min_length]
    # newSteer_data = newSteer_data[:min_length]
    # clipSteer_data = clipSteer_data[:min_length]


    # Plot 1: Yaw Calculation vs Time
    # axs[0].plot(time_data, imu_yaw_data, label="IMU Yaw", color="green")
    # axs[0].plot(time_data, yaw_calc_data, label="Yaw Calc", color="blue")
    axs[0].plot(time_data, error_data, label="Error", color="red")
    axs[0].plot(time_data, integral_data, label="Integral", color="orange")
    axs[0].plot(time_data, derivative_data, label="Derivative", color="purple")
    axs[0].set_ylabel("Yaw Values")
    axs[0].set_xlabel("Time (s)")
    axs[0].set_title("Yaw Calculation and IMU Yaw vs Time")
    axs[0].legend()
    axs[0].grid()

    # Plot 2: Error vs Time
    axs[1].plot(time_data, newSteer_data, label="Steering Angle", color="red")
    axs[1].plot(time_data, clipSteer_data, label="Clipped Angle", color="blue")
    axs[1].set_xlabel("Time (s)")
    axs[1].set_ylabel("Steering Angle")
    axs[1].set_title("Steering Angle vs Time")
    axs[1].legend()
    axs[1].grid()

    # Adjust layout
    plt.tight_layout(rect=[0, 0, 1, 0.96])  # Leave space for the suptitle
    # plt.savefig("combined_plot.png")  # Save the combined plot as a file
    plt.show()  # Display the plot

if __name__ == "__main__":
    main()
