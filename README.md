# stm32_mpr121
A MPR121 driver based on libopencm3 library for STM32F4 devices. Developed under arm-none-eabi-gcc toolchain.
The demo application is developed on an STM32F4 Discovery board with an MPR121 breakout board and turns the green LED ON if the MPR121 device is initialised successfully else the red LED. Then status of electrodes 1 and 11 are checked continuously and respectively blue and orange LEDs are turned ON if pressed.

For compiling the project it is necessary to download the libopencm3 library https://github.com/libopencm3/libopencm3. This project folder must be located in the same folder where libopencm3 is located. For compiling the project run:

    $ make

and for flashing the STM32F4Discovery board run:

    $ make flash

The default toolchain is the same of libopencm3, an arm-none-eabi/arm-elf toolchain.

**TODO**

Implement functions to use electrodes as GPIO and to configure electrodes parameters on init.
