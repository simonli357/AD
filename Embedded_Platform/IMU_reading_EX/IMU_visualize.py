import re
import matplotlib.pyplot as plt
from pathlib import Path
import numpy as np
from scipy.integrate import cumulative_trapezoid

m_velocityStationaryCounter = 0

def main():
    # Load and visualize data
    # 7 - x displacment mainly
    # 8 = y displacement mainly
    # 9 - z displacement mainly
    # 10 - diagonal and up
    filename = Path("c:/Users/linxy/Documents/BFMC/IMU_reading_EX/SWV_export/SWV_ITM_Data_Console_nomove.txt")
    # roll, pitch, yaw = parse_imu_data_euler(filename)
    # plot_euler_angles(roll, pitch, yaw)
    # plot_orientation_3d(roll, pitch, yaw)
    dt = 0.1  # Assuming a sampling rate of 10 Hz (dt = 1/10)
    # Step 1: Parse the data
    acc_data, euler_data = parse_imu_data_euler_acc_csv(filename)

    plot_acceleration_2d(acc_data)

    # Step 2: Transform acceleration to global coordinates
    acc_global = transform_acceleration(acc_data, euler_data)

    # Step 3: Compute the position
    position = compute_position(acc_global, dt)

    # Step 4: Plot the trajectory
    plot_trajectory(position)

def parse_imu_data_euler(filename):
    """Extracts Roll, Pitch, and Yaw from the provided IMU data file."""
    roll_list, pitch_list, yaw_list = [], [], []
    
    # Regular expression to match Roll, Pitch, Yaw lines
    euler_pattern = re.compile(r"\[\+\] Roll: ([\+\-]?\d+\.\d+) \| Pitch: ([\+\-]?\d+\.\d+) \| Yaw: ([\+\-]?\d+\.\d+)")
    
    with open(filename, 'r') as file:
        for line in file:
            match = euler_pattern.search(line)
            if match:
                roll, pitch, yaw = map(float, match.groups())
                roll_list.append(roll)
                pitch_list.append(pitch)
                yaw_list.append(yaw)
    
    return roll_list, pitch_list, yaw_list

def plot_euler_angles(roll, pitch, yaw):
    """Plots the Roll, Pitch, and Yaw angles."""
    plt.figure(figsize=(10, 6))
    
    plt.plot(roll, label='Roll', marker='o')
    plt.plot(pitch, label='Pitch', marker='x')
    plt.plot(yaw, label='Yaw', marker='s')
    
    plt.title('Euler Angles from IMU Data')
    plt.xlabel('Sample Number')
    plt.ylabel('Angle (Degrees)')
    plt.legend()
    plt.grid(True)
    plt.show()

def euler_to_rotation_matrix(roll, pitch, yaw):
    """Converts Euler angles to a rotation matrix."""
    # Convert angles from degrees to radians
    roll = np.deg2rad(roll)
    pitch = np.deg2rad(pitch)
    yaw = np.deg2rad(yaw)

    # Rotation matrices for each axis
    Rx = np.array([
        [1, 0, 0],
        [0, np.cos(roll), -np.sin(roll)],
        [0, np.sin(roll), np.cos(roll)]
    ])
    
    Ry = np.array([
        [np.cos(pitch), 0, np.sin(pitch)],
        [0, 1, 0],
        [-np.sin(pitch), 0, np.cos(pitch)]
    ])
    
    Rz = np.array([
        [np.cos(yaw), -np.sin(yaw), 0],
        [np.sin(yaw), np.cos(yaw), 0],
        [0, 0, 1]
    ])
    
    # Combined rotation matrix
    R = Rz @ Ry @ Rx
    return R

def plot_orientation_3d(roll, pitch, yaw):
    """Plots the orientation based on Roll, Pitch, Yaw in 3D space."""
    fig = plt.figure(figsize=(8, 8))
    ax = fig.add_subplot(111, projection='3d')
    
    # Set up axes limits
    ax.set_xlim([-1, 1])
    ax.set_ylim([-1, 1])
    ax.set_zlim([-1, 1])
    
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    
    for r, p, y in zip(roll, pitch, yaw):
        # Get rotation matrix from Euler angles
        R = euler_to_rotation_matrix(r, p, y)
        
        # Define axes of the object
        x_axis = R @ np.array([1, 0, 0])
        y_axis = R @ np.array([0, 1, 0])
        z_axis = R @ np.array([0, 0, 1])
        
        # Plot the orientation axes
        ax.quiver(0, 0, 0, x_axis[0], x_axis[1], x_axis[2], color='r', length=0.5, label='X-axis (Roll)')
        ax.quiver(0, 0, 0, y_axis[0], y_axis[1], y_axis[2], color='g', length=0.5, label='Y-axis (Pitch)')
        ax.quiver(0, 0, 0, z_axis[0], z_axis[1], z_axis[2], color='b', length=0.5, label='Z-axis (Yaw)')
    
    plt.title('3D Orientation Visualization using Euler Angles')
    plt.grid(True)
    plt.show()

def parse_imu_data_euler_acc_csv(filename):
    """
    Parses the IMU data from the provided .txt file.
    Returns numpy arrays for acceleration (ACC) and Euler angles (Roll, Pitch, Yaw).
    """
    acc_data = []
    euler_data = []
    
    with open(filename, 'r') as file:
        for line in file:
            parts = line.strip().split(',')
            label = parts[0]

            # Extract acceleration data (ACC)
            if label == "LIA":
                acc_data.append([float(parts[1]), float(parts[2]), float(parts[3])])
            
            # Extract Euler angles (Roll, Pitch, Yaw)
            elif label == "Roll":
                euler_data.append([float(parts[1]), float(parts[2]), float(parts[3])])

    # Convert lists to numpy arrays
    acc_data = np.array(acc_data)
    euler_data = np.array(euler_data)

    return acc_data, euler_data

def parse_imu_data_euler_acc_message(filename):
    """Extracts acceleration and Euler angles from the provided IMU data file."""
    acc_data = []
    euler_data = []
    
    # Regular expressions to match acceleration and Euler angles lines
    acc_pattern = re.compile(r"\[\+\] LIA - x: ([\+\-]?\d+\.\d+) \| y: ([\+\-]?\d+\.\d+) \| z: ([\+\-]?\d+\.\d+)")
    euler_pattern = re.compile(r"\[\+\] Roll: ([\+\-]?\d+\.\d+) \| Pitch: ([\+\-]?\d+\.\d+) \| Yaw: ([\+\-]?\d+\.\d+)")
    
    with open(filename, 'r') as file:
        for line in file:
            acc_match = acc_pattern.search(line)
            euler_match = euler_pattern.search(line)
            
            if acc_match:
                acc_x, acc_y, acc_z = map(float, acc_match.groups())
                acc_data.append([acc_x, acc_y, acc_z])
            
            if euler_match:
                roll, pitch, yaw = map(float, euler_match.groups())
                euler_data.append([roll, pitch, yaw])
    
    return np.array(acc_data), np.array(euler_data)

def transform_acceleration(acc, euler):
    """Transforms acceleration from the IMU frame to the global frame."""
    transformed_acc = []
    
    for i in range(len(acc)):
        roll, pitch, yaw = euler[i]
        R = euler_to_rotation_matrix(roll, pitch, yaw)
        
        global_acc = R @ acc[i]
        # # Subtract gravity from the Z-axis
        # global_acc[2] -= 9.81
        transformed_acc.append(global_acc)
    return transformed_acc

def compute_position(acc_global, dt): # acc_global is a list
    """Computes the position by double integrating the acceleration."""
    # Integrate acceleration to get velocity
    # stationary filter based on bfmc imu code:
    global m_velocityStationaryCounter
    velocity = []
    vel_x = 0
    vel_y = 0
    vel_z = 0
    
    for i in range(len(acc_global)):
        if all(-0.11 <= value <= 0.11 for value in acc_global[i]):
            m_velocityStationaryCounter+=1
            # for j in range(len(acc_global[i])):
            #     acc_global[i][j] = 0
            if m_velocityStationaryCounter==10:
                vel_x = 0
                vel_y = 0
                vel_z = 0
                m_velocityStationaryCounter=0
        else:
            vel_x = vel_x + acc_global[i][0]*dt/1000
            vel_y = vel_y + acc_global[i][1]*dt/1000
            vel_z = vel_z + acc_global[i][2]*dt/1000
            m_velocityStationaryCounter = 0
        velocity.append([vel_x,vel_y,vel_z])
    
    # velocity = cumulative_trapezoid(np.array(acc_global),dx=dt, initial=0, axis=0)/1000
    
    # Integrate velocity to get position
    position = cumulative_trapezoid(np.array(velocity), dx=dt, initial=0, axis=0)
    # position = cumulative_trapezoid((velocity), dx=dt, initial=0, axis=0)
    return position

def plot_trajectory(position):
    """Plots the 3D trajectory."""
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')
    
    ax.plot(position[:, 0], position[:, 1], position[:, 2], marker='o')
    ax.set_xlabel('X Position (m)')
    ax.set_ylabel('Y Position (m)')
    ax.set_zlabel('Z Position (m)')
    ax.set_title('3D Trajectory of IMU')
    plt.grid(True)
    plt.show()

def plot_acceleration_2d(acc_data, sampling_rate=10):
    """
    Plots the X, Y, Z acceleration data over time.
    
    Parameters:
    - acc_data: numpy array of shape (n, 3) where each row is [acc_x, acc_y, acc_z].
    - sampling_rate: the rate at which data was sampled (default: 10 Hz).
    """
    # Calculate the time vector based on the number of samples and the sampling rate
    num_samples = acc_data.shape[0]
    dt = 1 / sampling_rate  # Time interval between samples
    time = np.arange(0, (num_samples) * dt, dt)

    # Plot the acceleration data
    plt.figure(figsize=(10, 6))
    plt.plot(time, acc_data[:, 0], label='Acceleration X', marker='o')
    plt.plot(time, acc_data[:, 1], label='Acceleration Y', marker='x')
    plt.plot(time, acc_data[:, 2], label='Acceleration Z', marker='s')

    # Adding labels and title
    plt.xlabel('Time (s)')
    plt.ylabel('Acceleration (m/s²)')
    plt.title('Acceleration over Time')
    plt.legend()
    plt.grid(True)
    plt.show()


if __name__=="__main__":
    main()