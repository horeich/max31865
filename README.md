# Maxim Integrated MAX31865 RTD to digital converter
Hardware driver for mbedOS
## Required hardware
For prototyping the following hardware is recommended:
- [NUCLEO_L476RG](https://os.mbed.com/platforms/ST-Nucleo-L476RG/) - a bare-metal STM32 development board
- [MAX31865 Breakout-Board](https://www.google.com/search?client=firefox-b-d&q=MAX31865+breakout+board) - find your suitable board online

The library will work with any board supported by mbedOS. Enable RTOS for the best experience

## How to connect an RTD to MAX31865 device

For wiring refer to [RTD Wiring & Config](https://learn.adafruit.com/adafruit-max31865-rtd-pt100-amplifier/)

## Library features

- [x] Covers the entire functionality of the device
- [x] Non-blocking timeouts when RTOS is enabled

## System design (typical application)
### Hardware
![Main](https://github.com/horeich/max31865/blob/master/assets/schematics.jpg)
- Provide a capacitor between RTDIN+ and RTDIN- as a filter in noisy environments
- 
### Software
#### General

#### Faults
- The device supports *automatic fault detection* and *manual fault detection*
## Characteristics
- the device does not have an on-board EEPROM; the configuration has to be set after every power cycle



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

