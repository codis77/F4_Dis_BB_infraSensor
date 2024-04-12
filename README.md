# F4_Dis_BB_infraSensor

This is supposed to become an infrasound recording application, using the BMP280 pressure sensor
and the STM32F4 Discovery with the baseboard and LCD board attached.

 --- THIS IS WORK IN PROGRESS ---

While it compiles & and builds, it is not finished by far.
Only the basic sensor driver code is implemented, but not yet tested.
SD card storage and display functionality will be implemented soon.


The objective is to sample the air pressure at a 150Hz rate, store it on
a SD card if present, and display characteristics on the attached LCD display.

The code uses the ST legacy "standard peripheral library, and FatFS.
For this purpose, the toolchain header files and supportfiles are removed
from the project (removed from build), and replaced with SPL variants.

The Bosch BMP280 sensor is driven by SPI, and only raw pressure data
are collected. No compensation or calibration is applied, as they are
irrelevant for this purpose.


