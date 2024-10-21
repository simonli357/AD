# AD
Autonomous Driving Pipeline For Bosch Future Mobility Challenge: https://boschfuturemobility.com/

## Description

tbd

## Structure

- **control**: 
- **localization**: 
- **perception**: 
- **planning**: 
- **utils**: contains custom msgs and srvs used by the other packages.

## Dependencies

### ROS
#### Installation:
http://www.autolabor.com.cn/book/ROSTutorials/chapter1/12-roskai-fa-gong-ju-an-zhuang/124-an-zhuang-ros.html

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

## Running the Example

### Acados Simulation

1. Run the following command to execute the MPC simulation:
    ```bash
    python3 scripts/mpc_acados.py
    ```

2. Upon successful execution, a GIF illustrating the simulation will be saved in `scripts/gifs_acados`.

### Gazebo Simulation

#### In Python
##### Path Following
1. Start the gazebo simulation:
    ```bash
    source devel/setup.bash
    roslaunch sim_pkgs map_with_car.launch
    ```

2. Start the ekf node:
    ```bash
    roslaunch mpc ekf.launch
    ```
3. Start the control node:
    ```bash
    rosrun mpc control_node2.py --useEkf 
    ```

##### Follow Traffic Rules
1. Start the gazebo simulation:
    ```bash
    source devel/setup.bash
    roslaunch sim_pkgs map_with_some_objects.launch
    ```

2. Start the ekf node:
    ```bash
    roslaunch mpc ekf.launch
    ```
3. Start the object detctor node:
    ```bash
    roslaunch control signFastest.launch show:=true print:=true
    ```
3. Start the control node:
    ```bash
    rosrun mpc control_node2.py --useEkf --sign
    ```

#### In C++
##### Path Following
1. Start the gazebo simulation:
    ```bash
    source devel/setup.bash
    roslaunch sim_pkgs map2023.launch
    ```

2. Start the ekf node:
    ```bash
    roslaunch mpc ekf.launch
    ```
3. Start the control node:
    ```bash
    roslaunch mpc controller.launch ekf:=true
    ```
4. If ekf set to false, we use the ground truth from gazebo to get the vehicle pose. In this case, no need to launch the ekf node.

##### Follow Traffic Rules
1. Start the gazebo simulation:
    ```bash
    source devel/setup.bash
    roslaunch sim_pkgs map2023objects.launch
    ```

2. Start the ekf node:
    ```bash
    roslaunch mpc ekf.launch
    ```
3. Start the object detctor node:
    ```bash
    roslaunch control signFastest.launch show:=true print:=true
    ```
3. Start the control node:
    ```bash
    roslaunch mpc controller.launch ekf:=true sign:=true
    ```






