# nrf51_remote_pwm
Controlling two PWM channels through Nordic UART smartphone App. 
The two LEDs, LED3 and lED4, are controlled respectively by PWM CH1 and CH2. The PWM duty cycle of both channels varies from 0 to 100. These value are initialised at 0 and are increased up to 100 or decreased up to 0 by steps of 10 each. At every overflow values are reset to 0 and at every underflow they are set to 100.
The Nordic UART app must be configured to sen following strings:
 * "a+": for increasing PWM duty cycle value of the LED3 by 10;
 * "a-": for decreasing PWM duty cycle value of the LED3 by 10;
 * "b+": for increasing PWM duty cycle value of the LED4 by 10;
 * "b-": for decreasing PWM duty cycle value of the LED4 by 10.

This software has been derived from BLE UART and PWM examples from Nordic SDK and has been developed on top of softdevice s110 running on a nrf51 PCA10028 Dev. Kit.
You must have an arm-eabi-none GCC toolchain and JLink installed.


**Install**

Download Segger JLink tool from https://www.segger.com/jlink-software.html. Unpack it and move it to /opt directory.
Download the Nordic SDK from https://developer.nordicsemi.com/nRF51_SDK/nRF51_SDK_v9.x.x/. Unpack it and move it to /opt directory.
Clone my nrfjprog.sh repo in /opt directory by running:

    $ cd [your path]/opt
    $ git clone https://github.com/marcorussi/nrfjprog.git

Clone this repo in your projects directory:

    $ git clone https://github.com/marcorussi/nrf51_remote_pwm.git
    $ cd nrf51_remote_pwm
    $ gedit Makefile

Verify and modify following names and paths as required according to your ARM GCC toolchain:

```
PROJECT_NAME := nrf51_remote_pwm
NRFJPROG_PATH := /opt/nrfjprog
SDK_PATH := /opt/nRF51_SDK_9.0.0_2e23562
LINKER_SCRIPT := ble_remote_pwm_nrf51.ld
GNU_INSTALL_ROOT := /home/marco/ARMToolchain/gcc-arm-none-eabi-4_9-2015q2
GNU_VERSION := 4.9.3
GNU_PREFIX := arm-none-eabi
```


**Flash**

Connect your nrf51 Dev. Kit, make and flash it:
 
    $ make
    $ make flash_softdevice (for the first time only)
    $ make flash

You can erase the whole flash memory by running:

    $ make erase

