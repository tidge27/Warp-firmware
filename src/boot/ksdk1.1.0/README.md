
# Overview
This is the firmware for the bike cadence and wobble sensor.

## hardware
This firmware is tried and tested with the FRDM KL03 development board.  

In addition to the development board, the following devices are required:
- L3GD20H Gyro breakout board (by Adafruit)
- SSD1331 OLED Display breakout board (by Adafruit)

These are connected to the I2C and SPI bus respectively.  

- Hall effect sensor connected to PTA7

This Test setup could be miniaturised when a custom board is made.


### Pin connections:
##### Hall Effect:

```
signal - PTA7
```



##### Gyro:
```
SCL - PTB3
SDA - PTB4
```
##### Display:
```
OC   - PTB13
R    - PTB0
DC   - PTA12
CLK  - PTA9
MOSI - PTA8
```

## Source File Descriptions

##### `CMakeLists.txt`
This is the CMake configuration file. Edit this to change the default size of the stack and heap.

##### `SEGGER_RTT.*`
This is the implementation of the SEGGER Real-Time Terminal interface. Do not modify.

##### `SEGGER_RTT_Conf.h`
Configuration file for SEGGER Real-Time Terminal interface. You can increase the size of `BUFFER_SIZE_UP` to reduce text in the menu being trimmed.

##### `SEGGER_RTT_printf.c`
Implementation of the SEGGER Real-Time Terminal interface formatted I/O routines. Do not modify.

##### `devL3GD20H.*`
Driver for L3GD20H.

##### `devMMA8451Q.*`
Driver for MMA8451Q.

##### `devSSD1331.*`
Driver for SSD1331.

##### `gpio_pins.c`
Definition of I/O pin configurations using the KSDK `gpio_output_pin_user_config_t` structure.

##### `gpio_pins.h`
Definition of I/O pin mappings and aliases for different I/O pins to symbolic names relevant to the hardware design, via `GPIO_MAKE_PIN()`.

##### `startup_MKL03Z4.S`
Initialization assembler.

##### `warp-kl03-ksdk1.1-boot.c`
The core of the implementation.

This is where the interrupt handler is configured, to calculate the cadence when the hall sensor is triggered.  This cadence is saved to a global variable, which is then updated on the screen when the main program loop has finished reading the sensors.

This is also where the program loop takes readings from the gyro and accelerometer, which are then converted into an angle using an iterative algorithm.  The angle is then plotted on the display using the function in the display driver.  If the cadence has changed since the display was last updated, then the digits are updated.

##### `warp-kl03-ksdk1.1-powermodes.c`
Implements functionality related to enabling the different low-power modes of the KL03.

##### `warp.h`
Constant and data structure definitions.
