# STM32-MPU9250-I2C

Fork based on original [Original Arduino library](https://github.com/MarkSherstan/MPU-6050-9250-I2C-CompFilter) and [Original STM32 library](https://github.com/MarkSherstan/STM32-MPU6050-MPU9250-I2C-SPI). 

Library code was tested with the [Nucleo-F746ZG dev board](https://os.mbed.com/platforms/ST-Nucleo-F746zg/).

# Notes / Usage
* Download [STM32CubeIDE](https://www.st.com/en/development-tools/stm32cubeide.html) and create a new project based on your hardware (or import the example projects if the same dev board is available). 
* Select: Project Manager -> Code Generator -> Check: `Generate peripheral initialization as a pair of '.c/.h' files per peripheral`.
* Add provided desired header and source files into their respective `\Src` and `\Inc` directories.
* `#include "MPUXX50.h"` at the `main.h`
* Parameters required by library:
    - Hardware information (e.g. hi2cX or hspiX)
    - Gyroscope full scale range (default 500 deg/s)
    - Accelerometer full scale range (default 4 g)
    - Delta time (default 0.004 s (250 Hz) -> use a timer interrupt)
    - Time constant (default 0.98)

* The minimum functions required after initializing are: `begin()`, `calibrate()`, and `attitude()`
* Data can be printed from a serial port by connecting to the hardware

# Pinout
Refer to the  [here](https://os.mbed.com/platforms/ST-Nucleo-F746zg/).

## I2C
| MPU9250  	| STM32F401RE 	| Breakout board comment       	        |
|----------	|-------------	|-------------------------------------- |
| VDD      	| 3V3         	|                              	        |
| GND      	| GND         	|                              	        |
| SDA      	| PB9 (SDA)     |                              	        |
| SCL      	| PB6 (SCL)     |                              	        |
