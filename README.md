# AD
Autonomous Driving Pipeline For Bosch Future Mobility Challenge: https://boschfuturemobility.com/

## Description

This repository contains the Autonomous Driving (AD) pipeline developed for the Bosch Future Mobility Challenge. The project is designed for a 1-to-10 scale autonomous vehicle and includes modules for perception, localization, planning, and control. It leverages ROS (Robot Operating System) to manage communication between different components and integrates a variety of technologies, including model predictive control, extended Kalman filtering, and deep learning-based perception.

The pipeline is structured to operate both in simulation (Gazebo) and on real hardware, using an STM32-based embedded platform for low-level control. This pipeline serves as a robust framework for autonomous navigation, making it suitable for both competition and research purposes.


## Structure

- **Embedded_Platform**: code provided by Bosch and modified by us. Contains functionalities to interface with STM32, which controls the car's motor, servo and interacts with the sensors (e.g. IMU).
- **src/control**: Uses Model Predictive Control (MPC) for smooth path following and a state machine to handle autonomous decision-making.
- **src/localization**: Uses the `robot_localization` package to fuse sensor data via an Extended Kalman Filter (EKF) for state estimation.
- **src/perception**: Runs histogram based lane detection algorithm. Sign detection can use either custom trained yoloFastestV2 model deployed with NCNN (using CPU) or custom trained yoloV8 model deployed with TensorRT (using GPU). Detectable objects include traffic signs that can be encountered in the competition, traffic lights (+ color classification), vehicles, and the pedestrian doll.
- **src/planning**: Fetches global waypoints from graphML file, optimizes path using Spline interpolation.
- **src/utils**: Contains custom msgs and srvs used by the other packages.

## Dependencies

### Ubuntu 20.04

- Not sure if ROS noetic works on a different version.

#### Dual Boot:
https://linuxconfig.org/how-to-install-ubuntu-alongside-windows-11-dual-boot

### ROS
#### Installation:
http://www.autolabor.com.cn/book/ROSTutorials/chapter1/12-roskai-fa-gong-ju-an-zhuang/124-an-zhuang-ros.html

### Gazebo Simulator With RC Car and Track:
#### Installation:
https://github.com/simonli357/Simulator

### opencv (version 4.6.0 or above)
#### Installation:

1. Go to this link: https://docs.opencv.org/4.x/d7/d9f/tutorial_linux_install.html

2. Follow instructions in Build with opencv_contrib to install

3. Build cv_bridge:
    clone this repository if you haven't done so yet:
    ```bash
    cd
    git clone https://github.com/slsecrets357/AD.git
    ```
    locate where your opencv build, then build cv_bridge
    ```bash
    cd ~/AD
    # Replace /path/to/opencv-4.9.0 with the actual path to your OpenCV installation
    catkin_make -DOpenCV_DIR=/path/to/opencv-4.9.0/build
    ```

### Python libraries
#### Installation:
    ```bash
    pip install -r ~/AD/requirements.txt
    ```

### robot_localization
#### Installation:

1. Install it as follows:
    ```bash
    sudo apt update
    sudo apt install ros-noetic-robot-localization
    ```

### ncnn
#### Installation:

1. Go to this link: https://github.com/Tencent/ncnn/wiki/how-to-build

2. Follow instructions in to install for your specific machine.

3. Create an folder named "ncnn" in src/perception/include, then copy the "bin", "include" and "lib" folders from the installed ncnn directory into the new folder

### TensorRT
#### Installation:

1. Follow instructions in Cuda&TrtInstall.md to install.
2. add these lines in the cmakelist.txt to avoid NvInfer.h and cuda_runtime.h not found error. Replace with the actual path to your TensorRT and Cuda directories.
    ```bash
    include_directories(/home/{user}/TensorRT-8.6.1.6/include) 
    link_directories(/home/{user}/TensorRT-8.6.1.6/lib)
    include_directories(/usr/local/cuda/targets/x86_64-linux/include) 
    link_directories(/usr/local/cuda/targets/x86_64-linux/lib)"
    ```
### Librealsense

#### Installation:

1. Install dependencies:
```bash
    sudo apt-get update
    sudo apt-get install autoconf libudev-dev
```
2. Install vcpkg and librealsense2 with the following commands or follow instructions on https://github.com/IntelRealSense/librealsense:
```bash
    cd ~/AD
    git clone https://github.com/Microsoft/vcpkg.git
    cd vcpkg
    ./bootstrap-vcpkg.sh
    ./vcpkg integrate install
    ./vcpkg install realsense2
```

### Acados

#### Installation:

1. Clone the Acados repository and navigate into it:
    ```bash
    git clone https://github.com/acados/acados.git
    cd acados
    git submodule update --recursive --init
    ```

2. Create a build directory, navigate into it and run cmake:
    ```bash
    mkdir -p build
    cd build
    cmake .. -DACADOS_WITH_QPOASES=ON -DACADOS_EXAMPLES=ON -DHPIPM_TARGET=GENERIC -DBLASFEO_TARGET=ARMV8A_ARM_CORTEX_A57
    # Note: Replace ARMV8A_ARM_CORTEX_A57 with your device's architecture or use GENERIC if unsure.
    ```

3. Update the Makefile:
    ```bash
    # Set the following in <acados_root_folder>/Makefile.rule
    BLASFEO_TARGET = ARMV8A_ARM_CORTEX_A57
    ACADOS_WITH_QPOASES = 1
    ```

4. Build and install Acados:
    ```bash
    make -j4
    sudo make install -j4
    cd ..
    make shared_library
    ```

5. Install Python interface for Acados:
    ```bash
    pip3 install -e /home/{your user name}/acados/interfaces/acados_template
    ```

6. Update your `.bashrc` and source it:
    ```bash
    echo 'export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:"/home/{your user name}/acados/lib"' >> ~/.bashrc
    echo 'export ACADOS_SOURCE_DIR="/home/{your user name}/acados"' >> ~/.bashrc
    source ~/.bashrc
    ```

7. If you encounter issues with `t_renderer`, compile it natively:
    ```bash
    git clone https://github.com/acados/tera_renderer.git
    cd tera_renderer
    cargo build --verbose --release
    # Replace the file <acados_root_dir>/bin/t_renderer with the one compiled natively i.e. <tera_renderer_dir>/target/release/t_renderer
    ```

### Other Stuff
    ```bash
    sudo apt install nlohmann-json3-dev
    sudo apt-get install libncurses5-dev libncursesw5-dev
    ```

## Build

1. Build the packages using
    ```bash
    catkin_make --pkg utils
    catkin_make
    ```

## Usage

1. Run the simulation

2. Run the path planner server:
    ```bash
    source devel/setup.bash
    rosrun planning path2.py
    ```

3. Run the camera node in a new terminal:
    ```bash
    source devel/setup.bash
    roslaunch perception cameraNode.launch newlane:=false
    ```

4. Run the control node in a new terminal:
    ```bash
    source devel/setup.bash
    roslaunch control controller.launch sign:=true v:=25
    ```

4. Run the gui a new terminal:
    ```bash
    source devel/setup.bash
    rosrun perception gui.py
    ```
    Press start to follow the planned path as illustrated. To change the path, double click on a desired destination on the map, then press the goto button.




