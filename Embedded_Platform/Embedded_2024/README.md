# Embedded platform

Starter code from https://github.com/ECC-BFMC/Embedded_Platform

## Build process

(Requirements: [Python 3.6+](https://www.python.org/downloads/), [CMake 3.19.0+](https://cmake.org/download/))

1- Install Ninja and Mbed-tools:
```sh
pip install ninja
python -m pip install ninja
pip install mbed-tools
python -m pip install mbed-tools
```

2- Install cross-compiler from https://developer.arm.com/downloads/-/gnu-rm

3- Fetch Mbed library:
```sh
mbed-tools deploy
```

4- [Build and compile](https://os.mbed.com/docs/mbed-os/v6.16/build-tools/use.html):
```sh
mbed-tools compile -m B_L4S5I_IOT01A -t GCC_ARM
```

5- Flash the code on the board:
Copy cmake_build/B_L4S5I_IOT01A/develop/GCC_ARM/robot_car.bin into your stm board
