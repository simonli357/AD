#!/usr/bin/env python3

import serial
import time
import matplotlib
# matplotlib.use('Agg')  # Use a non-interactive backend
# import matplotlib.pyplot as plt

# On old car(2023): speed is in m/s
# Command - 4: Activate (send > 1) and deactivate (send 0) the PID
# Command - 5: Enable (send > 1) and disable (send 0) listening to the encoder (listen on serial port)
# LIMITS: speeds: {-0.5, 0.5} m/s, angles: {-25, 25} degrees

# On new car(2024): speed is in cm/s

# Functions to send commands
def speed(f_velocity):
    return "#{0}:{1:.2f};;\r\n".format(1, f_velocity)

def steer(f_angle):
    return "#{0}:{1:.2f};;\r\n".format(2, f_angle)

def both(f_velocity,f_angle):
    return "#{0}:{1:.2f}:{2:.2f};;\r\n".format(11, f_velocity, f_angle)

def PWM(f_velocity, f_angle):
    return "#{0}:{1:.10f}:{2:.10f};;\r\n".format(10, f_velocity, f_angle)

def Compute(f_velocity, f_angle):
    return "#{0}:{1:.2f}:{2:.2f};;\r\n".format(11, f_velocity, f_angle)

def setPID(f_active, f_proportional, f_integral, f_derivative):
    return "#{0}:{1:.4f}:{2:.4f}:{3:.4f}:{4:.4f};;\r\n".format(12, f_active, f_proportional, f_integral, f_derivative)

def main():
    # Serial port configuration
    serial_port = serial.Serial(
            '/dev/ttyACM0',
            baudrate=460800,     # match your firmware
            timeout=1,
            dsrdtr=False,        # disable DTR toggling
            rtscts=False         # disable RTS toggling
        )

    # Explicitly lower DTR/RTS in case the driver set them high by default
    serial_port.setDTR(False)
    serial_port.setRTS(False)

    # Data storage
    time_data = []
    yaw_calc_data = []
    imu_yaw_data = []
    error_data = []
    integral_data = []
    derivative_data = []

    try:
        # Send commands
        serial_port.write("#{0}:{1:.0f};;\r\n".format(4, 1).encode())
        # Tuned parameters : Kp = 1.25, Ki = 0.625, Kd = 15125;
        # setPID_msg = setPID(1, 1.25, 0.625, 0.15125)
        # print("PID ON msg sent:", setPID_msg)
        # serial_port.write(setPID_msg.encode())



        compute_msg = Compute(20.0, 0.0)
        print("Compute msg sent:", compute_msg)
        serial_port.write(compute_msg.encode())

        
        # compute_msg = PWM(0.0685, 0.0750)  
        # print("Compute msg sent:", compute_msg)
        # serial_port.write(compute_msg.encode())

        # time.sleep(20)

        # # compute_msg = PWM(0.081, 1.0)
        # compute_msg = Compute(0, -10.0)
        # print("Compute msg sent:", compute_msg)
        # serial_port.write(compute_msg.encode())

        # time.sleep(0)
        
        # compute_msg = PWM(0.077, 0.0)
        # print("Compute msg sent:", compute_msg)
        # serial_port.write(compute_msg.encode())

        # compute_msg = Compute(00.0, 00.0)
        # print("Compute msg sent:", compute_msg)
        # serial_port.write(compute_msg.encode())

        # setPID_msg = setPID(0, 0, 0, 0)
        # print("PID OFF msg sent:", setPID_msg)
        # serial_port.write(setPID_msg.encode())

    finally:
        serial_port.close()
        print("Serial port closed.")
    # both_msg = both(0.0, 0.0)
    # both_msg = both(30.0, 20)
    # print("both msg sent:", both_msg)
    # serial_port.write(both_msg.encode())


if __name__ == "__main__":
    main()
