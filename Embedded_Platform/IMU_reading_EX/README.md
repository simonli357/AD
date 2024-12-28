This is a STM32 project that should be running on STM32CubeMxIDE

The BNO055 driver code comes from GitHub repo: https://github.com/d-mironov/Bosch-BNO055-STM32

The SCL and SDA pin on the BFMC IMU board should be connected to pin 10 and 9 of CN1 on the STM32L4 board respectively.

The power of the IMU board should be connected to the 5V and GND pin of CN2 of the STM32L4 board.

The user button is used as the euler angle soft reset (i.e. not sensor reset) button.

The sensor calibration method structure is from this GitHub repo: https://github.com/Afebia/BNO055-STM32-V2/tree/bno055, then modified to accommodate with the driver used in this project.

If the sensor fails to init, replug the power of IMU and relaunch the project.