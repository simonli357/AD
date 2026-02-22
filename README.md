# AD: Autonomous Driving Pipeline

**Autonomous Driving Pipeline for the Bosch Future Mobility Challenge**  
[Official Challenge Website](https://boschfuturemobility.com/)
[Aftermovie 2025](https://www.youtube.com/watch?v=Z-UhcBtfA_4&ab_channel=BoschRomania)

---

## Description

This repository contains the Autonomous Driving (AD) pipeline developed for the Bosch Future Mobility Challenge. Designed for a 1:10 scale autonomous vehicle, the project includes core modules for:

- **Perception**: Object detection and lane recognition using deep learning.
- **Localization**: Sensor fusion with Extended Kalman Filtering (EKF).
- **Planning**: Path optimization and decision-making.
- **Control**: Model Predictive Control (MPC) for smooth trajectory execution.

Built on **ROS (Robot Operating System)**, the pipeline supports both **simulation (Gazebo)** and real-world deployment, leveraging an **STM32-based embedded platform** for low-level control.

---

<h2>Demo Videos</h2>

<div align="center">
  <a href="https://www.youtube.com/watch?v=LlrHEnVoWC4" target="_blank">
    <img src="https://img.youtube.com/vi/LlrHEnVoWC4/0.jpg" alt="Demo Video 1" width="75%" style="border-radius: 10px; box-shadow: 0 4px 8px rgba(0,0,0,0.3);">
    <p><em>BFMC 2025 Finals - Arena Run</em></p>
  </a>

  <a href="https://youtu.be/gsYTIXkuHi8" target="_blank">
    <img src="https://img.youtube.com/vi/bx6XyxSAWaU/0.jpg" alt="Demo Video 1" width="75%" style="border-radius: 10px; box-shadow: 0 4px 8px rgba(0,0,0,0.3);">
    <p><em>BFMC 2025 Qualification Video</em></p>
  </a>

  <a href="https://www.youtube.com/watch?v=xjAYdCH-X1w" target="_blank">
    <img src="https://img.youtube.com/vi/xjAYdCH-X1w/0.jpg" alt="Demo Video 2" width="75%" style="border-radius: 10px; box-shadow: 0 4px 8px rgba(0,0,0,0.3);">
    <p><em>Software v2.2.2 Simulator Demo</em></p>
  </a>
</div>

---

## Project Structure

- **Embedded_Platform/** - Interfaces with STM32 for motor, servo, and sensor control.
- **container/** - Nix container configuration files.
- **src/** - Contains the main modules:
  - **control/** - Implements Model Predictive Control (MPC) and state machine logic.
  - **localization/** - Fuses sensor data using an Extended Kalman Filter (EKF).
  - **perception/** - Uses deep learning models for object detection and lane recognition.
  - **planning/** - Generates optimized paths from global waypoints.
  - **gui/** - Provides visualization and manual control using PyQt5.
  - **utils/** - Defines custom ROS messages and services used by other packages.

Each of these modules is described in detail below, with figures where applicable.

---

## Detailed Package Overview

### Embedded Platform
This package contains firmware modified from Bosch’s provided code to interface with the **STM32 microcontroller**, controlling the vehicle’s **motor, servo, and sensors** (e.g., IMU, camera, encoders).

### 󱄅 Container
This package contains the nix flake for the development environment. To install, simply run the install script on any linux distribution. For Windows, install wsl2 and run the install script inside wsl2.

### Control
- Uses **Model Predictive Control (MPC)** for smooth trajectory tracking.
- Implements a **finite state machine** for handling autonomous behaviors.
![MPC Preview](assets/mpc_sim.gif)

### Localization
- Employs the `robot_localization` package to integrate GPS, IMU, and odometry data using an **Extended Kalman Filter (EKF)**.
- Simulated GPS delay and noise are added for realism in **Gazebo**.

### Perception
- **Lane Detection**: Uses a **histogram-based** approach.
- **Sign Detection**:
  - **YOLOFastestV2 using NCNN (CPU inference)** or **YOLOv8 using TensorRT (GPU inference)** for real-time inference.
  - Detects competition-relevant **traffic signs, traffic lights (with color classification), vehicles, and pedestrian dolls**.

### Planning
- Loads **global waypoints** from a **GraphML file**.
- Plans optimal path that goes through all desired destination points.
- Uses **spline interpolation** to generate a smooth, drivable path.

### GUI
- Built with **PyQt5** for real-time visualization of:
  - Camera feed
  - Vehicle state estimation
  - Planned paths and detected objects
- Provides manual override for:
  - **Start/stop controls**
  - **Path re-planning**
  - **Object toggling**
  - **Ros node launching**
  - **Real time object detection**
![MPC Preview](assets/gui.png)


---

## Dependencies

### Ubuntu 20.04
- Recommended OS. Not sure if ROS Noetic compatible with other versions.

### ROS Installation
- Follow the [ROS Installation Guide](http://www.autolabor.com.cn/book/ROSTutorials/chapter1/12-roskai-fa-gong-ju-an-zhuang/124-an-zhuang-ros.html).

### Simulation Platform
- Install the [Gazebo-based simulator](https://github.com/simonli357/Simulator).

### OpenCV (4.6.0+)
- Install using [this guide](https://docs.opencv.org/4.x/d7/d9f/tutorial_linux_install.html) with `opencv_contrib`.
- Build `cv_bridge`:
  ```bash
  catkin_make -DOpenCV_DIR=/path/to/opencv-4.9.0/build
  ```

### Required Python Libraries
- Install dependencies:
  ```bash
  pip install -r ~/AD/requirements.txt
  ```

### Robot Localization
- Install:
  ```bash
  sudo apt update && sudo apt install ros-noetic-robot-localization
  ```

### NCNN for YOLO Deployment
- Follow [this NCNN build guide](https://github.com/Tencent/ncnn/wiki/how-to-build).

### TensorRT for GPU Inference
- Follow `Cuda&TrtInstall.md` instructions for installation.
- Add paths to `CMakeLists.txt`:
  ```bash
  include_directories(/home/{user}/TensorRT-8.6.1.6/include)
  link_directories(/home/{user}/TensorRT-8.6.1.6/lib)
  ```

### Intel RealSense
- Install using [this guide](https://github.com/IntelRealSense/librealsense).
  ```bash
  sudo apt-get update && sudo apt-get install autoconf libudev-dev
  ```

### Acados for Optimization
- Follow [Acados installation steps](https://github.com/acados/acados) to install dependencies.
- Configure:
  ```bash
  echo 'export ACADOS_SOURCE_DIR="/home/{user}/acados"' >> ~/.bashrc
  ```

---

## Build Instructions

1. Build the packages:
  ```bash
  catkin_make --pkg utils
  catkin_make
  ```

---

## Usage

### Simulation

#### Start Simulation
- cd to where the simulator workspace is located.
```bash
source devel/setup.bash
roslaunch sim_pkg run132.launch
```
- rest is the same as real run.

### Real Vehicle Run

#### Start GUI
```bash
rosrun gui main.py
```
- run this on another computer to see what the car is doing.

#### Activate ROS master
- ssh to the jetson orin nano and run roscore:
```bash
ssh scandy@{ip_address_of_jetson}
roscore
```
#### Run Control Node
- ssh to the jetson orin nano and run the controller:

```bash
ssh scandy@{ip_address_of_jetson}
roslaunch control controller.launch camera:=true ip:=192.168.50.216 use_gps:=true real:=true realsense:=true use_traffic_server:=true gps_id:=10


roslaunch control controller.launch camera:=true  use_gps:=false real:=true realsense:=true use_traffic_server:=false gps_id:=10 print:=true emergency:=false
```
- replace 192.168.50.216 by ip address of computer on which the gui is run.
- change gps_id according to the id of the gps module mounted on the car.

#### Start the Run

- if gps is used, the car should autonomously plan a path from the starting point to all destination points.
- once the path is visible on the gui, press the start button to start the run.
- if gps is not used, click on any destination point on the map in the gui, press the plan button to plan a path, then press the start button to start the run.


