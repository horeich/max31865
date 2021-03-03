# Maxim Integrated MAX31865 RTD to digital converter
Hardware driver for mbedOS
## Required hardware
For prototyping the following hardware is recommended:
- [e.g. NUCLEO_L476RG](https://os.mbed.com/platforms/ST-Nucleo-L476RG/) - a bare-metal STM32 development board
- [e.g. Adafruit MAX31865](https://learn.adafruit.com/adafruit-max31865-rtd-pt100-amplifier) - MAX31865 breakout board (there are cheaper versions available)
- [e.g. AT102 PT100](https://autosen.com/en/Process-Sensors/Resistance-temperature-sensor-PT1000-AT102) - 4-wire temperature sensor

The library will work with any board supported by mbedOS. Enable RTOS for the best experience

## How to connect an RTD to MAX31865 device

For wiring refer to [RTD Wiring & Config](https://learn.adafruit.com/adafruit-max31865-rtd-pt100-amplifier/)

## Library features

- [x] Covers the entire functionality of the device
- [x] Non-blocking timeouts when RTOS is enabled

## System design (typical application)

### Interfacing with 1.8V MCU
- Maximum difference between MCU and MAX DVDD should be 1.5V. When using
### Hardware
![schematics](https://github.com/horeich/max31865/blob/master/assets/schematics.png)
- Provide a capacitor between RTDIN+ and RTDIN- as a filter in noisy environments
- Do not solder any jumpers on the breakout board for the 4-wire setup
- It is recommended to use an external pullup resistor for the *DRDY*-pin (indicating a finished conversion)
- Do not use any pull-ups on SDI/SDO or SCLK line, they do not have any influcence on the data communication, SPI is internally switched with push-pull logic
- A finished conversion is indicated by a falling edge

### Software
- The device has only 8 bit registers
#### General

#### Faults
- Faults are only supported in one-shot conversion mode (*NORMALLY OFF* bit 6 in config register must be set)
- The device supports *automatic fault detection* and *manual fault detection*; the bits 2 and 3 must be set in the configuration register
- Faults are encoded in and read from the *Fault Status Register 0x07* (each bit of the register represents one fault)
## Characteristics
- the device does not have an on-board EEPROM; the configuration has to be set after every power cycle; use power-cylce for hard-resetting the device



## Examples

### General
- Configure the pins (MOSI, MISO, CLK, CS, RDY) in the mbed_lib.json file
- Also, configure the nominal resistance of the RTD (typically 100Ω or 1000Ω) and matching resistance in the mbed_lib.json file

### max31865_oneshot.cpp
This example shows how to set up the device in one-shot mode

## Known issues
- the time constant in data sheet depends on the external RTD interface circuitry which is not accounted for. Suggestions to implement that are appreciated
## References
1. Datasheet Maxim Integrated MAX31865 RTD-to-Digital Converter [[1]](https://datasheets.maximintegrated.com/en/ds/MAX31865.pdf)

